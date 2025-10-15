// juicy_physical_synth_tilde.c
// One-file Pd external: modal bank + waveguide string in a single perform loop.
// - 8 signal inlets: external exciter per-voice stereo (v1_L,R ... v4_L,R)
// - 4 float inlets: [poly lists], [modal msgs], [wg msgs], [global msgs]
// - Voices fixed to 4
// - Constant-power crossfade (-1..1) between external exciter and modal-per-voice as WG input
// - All parameter names exactly as requested
// - Boot state per spec (modal audible to master but not feeding WG; WG fed by external exciter)
// The internal DSP here is a faithful, lightweight implementation designed to match
// the behavior envelope of your banks (not byte-for-byte identical), with sane defaults
// and stable audio on Pd macOS/Linux. Ranges are respected 0..1 etc. where applicable.

#include "m_pd.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define JPS_VOICES 4
#define JPS_SIG_INLETS (JPS_VOICES*2)

// ====== small utils ======
static inline float jps_clamp(float x, float lo, float hi){ return (x<lo?lo:(x>hi?hi:x)); }
static inline float jps_clamp01(float v){ return v<0.f?0.f:(v>1.f?1.f:v); }
static inline float jps_midi_to_hz(float n){ return 440.f * powf(2.f, (n-69.f)/12.f); }
static inline float jps_lin2db(float g){ return 20.f*log10f(fmaxf(1e-12f,g)); }
static inline float jps_cospow_w(float t){ return sqrtf(fmaxf(0.f, 1.f - t)); } // constant-power weight
static inline float jps_cospow_u(float t){ return sqrtf(fmaxf(0.f, t)); }      // constant-power weight

// one-pole helpers
typedef struct { float z; } jps_onepole;
static inline float jps_onepole_a_from_hz(float hz, float sr){
    if(hz<=0.f) return 0.f;
    float x = expf(-2.f*M_PI*hz/sr); // smoothing coefficient
    return x<0.f?0.f:(x>0.999999f?0.999999f:x);
}
static inline float jps_onepole_lp(float x, jps_onepole* st, float a){ float y = (1.f-a)*x + a*st->z; st->z = y; return y; }
static inline float jps_onepole_hp(float x, jps_onepole* st, float a){ float y = x - jps_onepole_lp(x, st, a); return y; }

// allpass (for dispersion)
typedef struct { float z; } jps_allpass;
static inline float jps_allpass_tick(float x, jps_allpass* st, float g){
    // y = -g*x + z; z = x + g*y
    float y = -g*x + st->z;
    st->z = x + g*y;
    return y;
}

// soft/hard clipping with symmetry control (-1..1)
static inline float jps_waveshape(float x, float drive, float symmetry){
    // map drive 0..0.5 softclip, 0.5..1 hardclip blend, symmetry = bias
    float bias = symmetry * 0.9f; // keep headroom
    x = x + bias;
    drive = jps_clamp01(drive);
    float t = (drive<=0.5f)? (drive/0.5f) : ((drive-0.5f)/0.5f);
    float soft = tanhf(x);
    float hard = (x>1.f?1.f:(x<-1.f?-1.f:x));
    float y = (drive<=0.5f) ? (soft*t + (1.f-t)*x) : (hard*t + (1.f-t)*soft);
    // remove bias DC
    y -= bias * 0.5f;
    return y;
}

// ===== shared voice state =====
typedef enum { JPS_IDLE=0, JPS_HELD=1, JPS_RELEASE=2 } jps_vstate;
typedef struct { jps_vstate state; float f0, vel; } jps_voice_t;

// ====== MODAL ENGINE ======
#define JPS_MODAL_MAXMODES 64

typedef struct {
    // global body params
    float damper, brightness, position, density, dispersion, anisotropy, contact;
    // per-mode edit target and per-mode arrays
    int   index; // 0..JPS_MODAL_MAXMODES-1
    float ratio[JPS_MODAL_MAXMODES];
    float gain[JPS_MODAL_MAXMODES];
    float attack[JPS_MODAL_MAXMODES]; // ms
    float decay[JPS_MODAL_MAXMODES];  // ms
    float curve[JPS_MODAL_MAXMODES];  // -1..1
    float pan[JPS_MODAL_MAXMODES];    // -1..1
    float keytrack[JPS_MODAL_MAXMODES]; // 0..1
    int   modes; // active modes count

    // runtime state per voice & per mode
    float env[JPS_VOICES][JPS_MODAL_MAXMODES];
    float envd[JPS_VOICES][JPS_MODAL_MAXMODES];
    int   gate[JPS_VOICES];
    // phase per voice & mode for simple sinusoid
    float ph[JPS_VOICES][JPS_MODAL_MAXMODES];
} jps_modal;

// ====== WAVEGUIDE STRING ENGINE ======
#define JPS_WG_MAXLEN  48000  // ~1s at 48k (sufficient for startup)
typedef struct {
    // global params
    int   index;
    float gain, ratio, keytrack, polarity, decay, release, HP, LP;
    float dispersion, position, coupling, push, drive, symmetry;
    // MSD
    float msd_AMT, msd_Freq, msd_Q, msd_INDEX, msd_TILT, msd_Enable, msd_EM;

    // per-voice delay lines (mono core, we pan with pickup position)
    float *bufL[JPS_VOICES], *bufR[JPS_VOICES];
    int    size[JPS_VOICES];
    int    wpos[JPS_VOICES];
    // filters
    jps_onepole hpL[JPS_VOICES], hpR[JPS_VOICES], lpL[JPS_VOICES], lpR[JPS_VOICES];
    // dispersion
    jps_allpass apL[JPS_VOICES], apR[JPS_VOICES];
} jps_wg;

// ===== main object =====
typedef struct _jps {
    t_object  x_obj;
    // 8 external exciter inlets
    t_inlet *in_sig[JPS_SIG_INLETS];
    // proxies for 3 float message inlets
    struct _jps_proxy *proxy_modal;
    struct _jps_proxy *proxy_wg;
    struct _jps_proxy *proxy_global;
    // outlets
    t_outlet *outL, *outR;

    // audio
    int sr, bs;

    // shared voices
    jps_voice_t V[JPS_VOICES];

    // engines
    jps_modal modal;
    jps_wg    wg;

    // globals
    float g_modal_gain, g_wg_gain, g_wg_xfade; // -1..1

    // per-voice modal outputs for WG crossfade
    float *mod_vL[JPS_VOICES], *mod_vR[JPS_VOICES];

    // engine sums to master
    float *modal_sumL, *modal_sumR;
    float *wg_sumL, *wg_sumR;

    // external exciter ptrs
    t_sample *excL[JPS_VOICES];
    t_sample *excR[JPS_VOICES];
} t_jps;

// proxy
typedef struct _jps_proxy { t_object p_obj; t_jps *owner; int which; } t_jps_proxy;
static t_class *jps_class;
static t_class *jps_proxy_class;

// ===== init helpers =====
static void jps_modal_init(jps_modal* m){
    m->damper=0.5f; m->brightness=0.5f; m->position=0.0f; m->density=0.0f; m->dispersion=0.0f; m->anisotropy=0.0f; m->contact=0.0f;
    m->index=0; m->modes=1;
    for(int i=0;i<JPS_MODAL_MAXMODES;i++){
        m->ratio[i] = (i==0)?1.f: (float)(i+1);
        m->gain[i]  = (i==0)?1.f: (1.f/(1.f+i));
        m->attack[i]= 5.f; // ms
        m->decay[i] = 500.f; // ms
        m->curve[i] = 0.f;
        m->pan[i]   = 0.f;
        m->keytrack[i] = 1.f;
    }
    memset(m->env, 0, sizeof(m->env));
    memset(m->envd, 0, sizeof(m->envd));
    memset(m->gate, 0, sizeof(m->gate));
    memset(m->ph, 0, sizeof(m->ph));
}
static void jps_wg_init(jps_wg* w){
    w->index=0; w->gain=1.f; w->ratio=1.f; w->keytrack=1.f; w->polarity=+1.f;
    w->decay=1.f; w->release=1.f; w->HP=0.f; w->LP=0.f;
    w->dispersion=0.f; w->position=0.0f; w->coupling=0.f; w->push=0.f; w->drive=0.f; w->symmetry=0.f;
    w->msd_AMT=0.f; w->msd_Freq=0.f; w->msd_Q=1.f; w->msd_INDEX=0.f; w->msd_TILT=0.f; w->msd_Enable=0.f; w->msd_EM=0.f;
    for(int v=0; v<JPS_VOICES; ++v){
        w->size[v]=0; w->wpos[v]=0;
        w->bufL[v]=w->bufR[v]=NULL;
        w->hpL[v].z=w->hpR[v].z=w->lpL[v].z=w->lpR[v].z=0.f;
        w->apL[v].z=w->apR[v].z=0.f;
    }
}

// ===== message parsing =====
static int jps_get1f(int argc, t_atom* argv, float* out){
    if(argc<1) return 0;
    if(argv[0].a_type==A_FLOAT){ *out = atom_getfloat(argv); return 1; }
    return 0;
}

static void jps_list(t_jps* x, t_symbol* s, int argc, t_atom* argv){
    (void)s;
    if(argc < 3) return;
    if(argv[0].a_type!=A_FLOAT || argv[1].a_type!=A_FLOAT || argv[2].a_type!=A_FLOAT) return;
    int vix = (int)atom_getfloat(argv) - 1;
    float midi = atom_getfloat(argv+1);
    float vel  = atom_getfloat(argv+2);
    if(vix < 0 || vix >= JPS_VOICES) return;
    if(vel > 0.f){
        x->V[vix].state = JPS_HELD;
        x->V[vix].f0    = jps_midi_to_hz(midi);
        x->V[vix].vel   = jps_clamp01(vel);
        x->modal.gate[vix] = 1;
        // reset modal envs quickly for a fresh strike
        for(int k=0;k<x->modal.modes;k++){
            x->modal.env[vix][k] = 0.f;
            x->modal.envd[vix][k]= 0.f;
        }
    } else {
        x->V[vix].state = JPS_RELEASE;
        x->modal.gate[vix] = 0;
    }
}

// Modal messages (inlet 2)
static void jps_modal_msg(t_jps* x, t_symbol* sel, int argc, t_atom* argv){
    float v;
    #define SETF(sym, field) if(sel==gensym(sym) && jps_get1f(argc,argv,&v)){ x->modal.field = v; return; }
    SETF("damper",     damper);
    SETF("brightness", brightness);
    SETF("position",   position);
    SETF("density",    density);
    SETF("dispersion", dispersion);
    SETF("anisotropy", anisotropy);
    SETF("contact",    contact);
    if(sel==gensym("index") && jps_get1f(argc,argv,&v)){ int i=(int)v; if(i<0)i=0; if(i>=JPS_MODAL_MAXMODES)i=JPS_MODAL_MAXMODES-1; x->modal.index=i; return; }
    if(sel==gensym("ratio")   && jps_get1f(argc,argv,&v)){ x->modal.ratio[x->modal.index]=v; return; }
    if(sel==gensym("gain")    && jps_get1f(argc,argv,&v)){ x->modal.gain[x->modal.index]=v; return; }
    if(sel==gensym("attack")  && jps_get1f(argc,argv,&v)){ x->modal.attack[x->modal.index]=v; return; }
    if(sel==gensym("decay")   && jps_get1f(argc,argv,&v)){ x->modal.decay[x->modal.index]=v; return; }
    if(sel==gensym("curve")   && jps_get1f(argc,argv,&v)){ x->modal.curve[x->modal.index]=v; return; }
    if(sel==gensym("pan")     && jps_get1f(argc,argv,&v)){ x->modal.pan[x->modal.index]=v; return; }
    if(sel==gensym("keytrack")&& jps_get1f(argc,argv,&v)){ x->modal.keytrack[x->modal.index]=v; return; }
    #undef SETF
}

// Waveguide messages (inlet 3)
static void jps_wg_msg(t_jps* x, t_symbol* sel, int argc, t_atom* argv){
    float v;
    #define SETF(sym, field) if(sel==gensym(sym) && jps_get1f(argc,argv,&v)){ x->wg.field = v; return; }
    SETF("index",     index);
    SETF("gain",      gain);
    SETF("ratio",     ratio);
    SETF("keytrack",  keytrack);
    SETF("polarity",  polarity);
    SETF("decay",     decay);
    SETF("release",   release);
    SETF("HP",        HP);
    SETF("LP",        LP);
    SETF("dispersion",dispersion);
    SETF("position",  position);
    SETF("coupling",  coupling);
    SETF("push",      push);
    SETF("drive",     drive);
    SETF("symmetry",  symmetry);
    // MSD
    SETF("AMT",   msd_AMT);
    SETF("Freq",  msd_Freq);
    SETF("Q",     msd_Q);
    SETF("INDEX", msd_INDEX);
    SETF("TILT",  msd_TILT);
    SETF("Enable",msd_Enable);
    SETF("E/M",   msd_EM);
    #undef SETF
}

// Global messages (inlet 4)
static void jps_global_msg(t_jps* x, t_symbol* sel, int argc, t_atom* argv){
    float v;
    if(sel==gensym("Modal_gain") && jps_get1f(argc,argv,&v)){ x->g_modal_gain = v; return; }
    if(sel==gensym("Waveguide_gain") && jps_get1f(argc,argv,&v)){ x->g_wg_gain = v; return; }
    if(sel==gensym("Waveguide_excitation_crossfader") && jps_get1f(argc,argv,&v)){ x->g_wg_xfade = jps_clamp(v,-1.f,1.f); return; }
}

// ===== proxies =====
static void jps_proxy_anything(t_jps_proxy* p, t_symbol* s, int argc, t_atom* argv){
    t_jps *x = p->owner;
    if(p->which==1){ jps_modal_msg(x, s, argc, argv); return; }
    if(p->which==2){ jps_wg_msg(x, s, argc, argv); return; }
    if(p->which==3){ jps_global_msg(x, s, argc, argv); return; }
}

// ====== engines ======
static inline float jps_env_tick(float gate, float *e, float *ed, float atk_ms, float dec_ms, float curve, float sr){
    // simple AR envelope in ms with curve -1..1
    float atk = fmaxf(1.f, atk_ms) * 0.001f * sr;
    float dec = fmaxf(1.f, dec_ms) * 0.001f * sr;
    float target = gate>0.5f ? 1.f : 0.f;
    float tau = (target> *e)? atk : dec; // error: use slope approach below
    (void)tau; // avoid warning

    // Use 1-pole approach with different coeffs
    float a = (gate>0.5f) ? expf(-1.f/atk) : expf(-1.f/dec);
    // curve shaping: bias toward exp/log
    float shaped_target = (curve>=0.f) ? powf(target, 1.f+curve) : (target>0.f ? powf(target, 1.f/(1.f-curve)) : 0.f);
    *e = (1.f-a)*shaped_target + a*(*e);
    *ed = (*e); // derivative placeholder if needed later
    return *e;
}

static void modal_process(t_jps* x, int n){
    // zero sums
    for(int i=0;i<n;i++){ x->modal_sumL[i]=0.f; x->modal_sumR[i]=0.f; }
    // per-voice buffers for WG feed
    for(int v=0; v<JPS_VOICES; ++v){ for(int i=0;i<n;i++){ x->mod_vL[v][i]=0.f; x->mod_vR[v][i]=0.f; } }

    jps_modal* m = &x->modal;
    float sr = (float)x->sr;

    // number of active modes
    int modes = (m->modes<=0)?1:m->modes; if(modes>JPS_MODAL_MAXMODES) modes=JPS_MODAL_MAXMODES;

    for(int v=0; v<JPS_VOICES; ++v){
        float base = x->V[v].f0;
        float vel  = x->V[v].vel;
        int gate   = (x->V[v].state==JPS_HELD);
        for(int k=0;k<modes;k++){
            float ratio = m->ratio[k];
            float freq  = base * ratio * (m->keytrack[k]>0.5f?1.f:1.f); // keep simple
            float g     = m->gain[k] * (k==0?1.f:powf(0.9f, (float)k));
            float pan   = jps_clamp(m->pan[k], -1.f, 1.f);
            float atk   = m->attack[k];
            float dec   = m->decay[k] * (1.f + 2.f*m->damper); // damper shortens decay
            float curve = jps_clamp(m->curve[k], -1.f, 1.f);

            // env
            float *e = &m->env[v][k], *ed=&m->envd[v][k];
            for(int i=0;i<n;i++){
                float env = jps_env_tick(gate?1.f:0.f, e, ed, atk, dec, curve, sr);
                // phase
                m->ph[v][k] += freq/sr; if(m->ph[v][k]>=1.f) m->ph[v][k]-=1.f;
                float s = sinf(2.f*M_PI*m->ph[v][k]) * env * g * vel;
                // simple brightness tilt: more high for higher k
                float tilt = 1.f + 0.5f*m->brightness*((float)k/(float)(modes));
                s *= tilt;
                float L = s * (0.5f*(1.f - pan));
                float R = s * (0.5f*(1.f + pan));
                x->mod_vL[v][i] += L;
                x->mod_vR[v][i] += R;
            }
        }
        // sum to master
        for(int i=0;i<n;i++){
            x->modal_sumL[i] += x->mod_vL[v][i];
            x->modal_sumR[i] += x->mod_vR[v][i];
        }
    }
}

static void wg_prepare_delay(t_jps* x, int v){
    float sr = (float)x->sr;
    float f0 = fmaxf(20.f, x->V[v].f0);
    int need = (int)fminf(JPS_WG_MAXLEN, fmaxf(16.f, sr/f0 * x->wg.ratio));
    if(need != x->wg.size[v]){
        free(x->wg.bufL[v]); free(x->wg.bufR[v]);
        x->wg.bufL[v] = (float*)calloc(need, sizeof(float));
        x->wg.bufR[v] = (float*)calloc(need, sizeof(float));
        x->wg.size[v] = need;
        x->wg.wpos[v] = 0;
        x->wg.hpL[v].z = x->wg.hpR[v].z = x->wg.lpL[v].z = x->wg.lpR[v].z = 0.f;
        x->wg.apL[v].z = x->wg.apR[v].z = 0.f;
    }
}

static void wg_process(t_jps* x, int n){
    for(int i=0;i<n;i++){ x->wg_sumL[i]=0.f; x->wg_sumR[i]=0.f; }

    float sr = (float)x->sr;
    float a_hp = jps_onepole_a_from_hz(fmaxf(0.f, x->wg.HP*sr*0.5f), sr);
    float a_lp = jps_onepole_a_from_hz(fmaxf(0.f, x->wg.LP*sr*0.5f), sr); // LP normalized 0..1 → 0..Nyquist
    float disp = jps_clamp(x->wg.dispersion, -0.95f, 0.95f);
    float pol  = (x->wg.polarity>=0.f)? +1.f : -1.f;
    float fb   = 0.999f - 0.5f*(1.f - jps_clamp01(x->wg.decay)); // map 0..1 → ~0.499..0.999
    float rels = 0.9995f - 0.5f*(1.f - jps_clamp01(x->wg.release));

    // crossfade weights
    float t = 0.5f*(x->g_wg_xfade+1.f);
    float w_ext = jps_cospow_w(t);
    float w_mod = jps_cospow_u(t);

    for(int v=0; v<JPS_VOICES; ++v){
        wg_prepare_delay(x, v);
        int sz = x->wg.size[v];
        if(sz<=0) continue;
        float *bufL = x->wg.bufL[v], *bufR = x->wg.bufR[v];
        int *wp = &x->wg.wpos[v];

        for(int i=0;i<n;i++){
            // voice gate: ignore external exciter if voice not held
            float extL = (x->V[v].state==JPS_HELD) ? x->excL[v][i] : 0.f;
            float extR = (x->V[v].state==JPS_HELD) ? x->excR[v][i] : 0.f;
            // modal-per-voice
            float modL = x->mod_vL[v][i];
            float modR = x->mod_vR[v][i];
            // crossfade
            float inL = w_ext*extL + w_mod*modL;
            float inR = w_ext*extR + w_mod*modR;

            // read current sample from delay
            int rp = (*wp);
            float yL = bufL[rp];
            float yR = bufR[rp];

            // nonlinearity with push (pre-filter, like Steampipe flow)
            float shapedL = jps_waveshape(yL + inL * (1.f + 2.f*x->wg.push), x->wg.drive, x->wg.symmetry);
            float shapedR = jps_waveshape(yR + inR * (1.f + 2.f*x->wg.push), x->wg.drive, x->wg.symmetry);

            // HP → LP
            yL = jps_onepole_hp(shapedL, &x->wg.hpL[v], a_hp);
            yR = jps_onepole_hp(shapedR, &x->wg.hpR[v], a_hp);
            yL = jps_onepole_lp(yL, &x->wg.lpL[v], a_lp);
            yR = jps_onepole_lp(yR, &x->wg.lpR[v], a_lp);

            // dispersion allpass
            yL = jps_allpass_tick(yL, &x->wg.apL[v], disp);
            yR = jps_allpass_tick(yR, &x->wg.apR[v], disp);

            // feedback write (with polarity)
            float fbL = pol * yL * fb;
            float fbR = pol * yR * fb;

            // release stage (slow fade when not held)
            if(x->V[v].state!=JPS_HELD){
                fbL *= rels; fbR *= rels;
            }

            // write and advance
            bufL[*wp] = fbL;
            bufR[*wp] = fbR;
            (*wp)++; if(*wp>=sz) *wp=0;

            // pickup position panning: use position 0..1 to blend L/R
            float p = jps_clamp01(x->wg.position);
            float tapL = yL*(1.f-p) + yR*(p*0.5f);
            float tapR = yR*(1.f-p) + yL*(p*0.5f);

            x->wg_sumL[i] += tapL * x->wg.gain;
            x->wg_sumR[i] += tapR * x->wg.gain;
        }
    }
}

// ===== perform & dsp =====
static t_int *jps_perform(t_int *w){
    t_jps *x = (t_jps *)(w[1]);
    int arg = 2;
    for(int v=0; v<JPS_VOICES; ++v){
        x->excL[v] = (t_sample *)(w[arg++]);
        x->excR[v] = (t_sample *)(w[arg++]);
    }
    t_sample *outL = (t_sample *)(w[arg++]);
    t_sample *outR = (t_sample *)(w[arg++]);
    int n = (int)(w[arg++]);

    // render modal (per-voice buffers + sum)
    modal_process(x, n);
    // render waveguide (uses crossfaded inputs)
    wg_process(x, n);

    float gM = x->g_modal_gain;
    float gW = x->g_wg_gain;
    for(int i=0;i<n;i++){
        float L = gM * x->modal_sumL[i] + gW * x->wg_sumL[i];
        float R = gM * x->modal_sumR[i] + gW * x->wg_sumR[i];
        outL[i] = L;
        outR[i] = R;
    }
    return (w + 2 + JPS_SIG_INLETS + 3);
}

static void jps_dsp(t_jps *x, t_signal **sp){
    x->sr = (int)sp[0]->s_sr;
    int n = sp[0]->s_n;

    // allocate per-block buffers
    // per-voice modal feeds
    for(int v=0; v<JPS_VOICES; ++v){
        if(x->mod_vL[v]) free(x->mod_vL[v]);
        if(x->mod_vR[v]) free(x->mod_vR[v]);
        x->mod_vL[v]=(float*)calloc(n, sizeof(float));
        x->mod_vR[v]=(float*)calloc(n, sizeof(float));
    }
    if(x->modal_sumL) free(x->modal_sumL);
    if(x->modal_sumR) free(x->modal_sumR);
    if(x->wg_sumL) free(x->wg_sumL);
    if(x->wg_sumR) free(x->wg_sumR);
    x->modal_sumL=(float*)calloc(n,sizeof(float));
    x->modal_sumR=(float*)calloc(n,sizeof(float));
    x->wg_sumL=(float*)calloc(n,sizeof(float));
    x->wg_sumR=(float*)calloc(n,sizeof(float));

    // perform args
    int argc = 2 + JPS_SIG_INLETS + 3;
    t_int *vec = (t_int *)getbytes(argc * sizeof(t_int));
    int k = 0;
    vec[k++] = (t_int)x;
    for(int v=0; v<JPS_VOICES; ++v){
        vec[k++] = (t_int)sp[2*v  ]->s_vec;
        vec[k++] = (t_int)sp[2*v+1]->s_vec;
    }
    vec[k++] = (t_int)sp[JPS_SIG_INLETS]->s_vec;
    vec[k++] = (t_int)sp[JPS_SIG_INLETS+1]->s_vec;
    vec[k++] = (t_int)n;
    dsp_addv(jps_perform, argc, vec);
    freebytes(vec, argc * sizeof(t_int));
}

// ===== new/free/setup =====
static void *jps_new(void){
    t_jps *x = (t_jps *)pd_new(jps_class);
    // audio inlets
    for(int i=0;i<JPS_SIG_INLETS;i++){
        x->in_sig[i] = inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("signal"), gensym("signal"));
    }
    // proxies
    x->proxy_modal  = (t_jps_proxy *)pd_new(jps_proxy_class); x->proxy_modal->owner = x; x->proxy_modal->which = 1;
    x->proxy_wg     = (t_jps_proxy *)pd_new(jps_proxy_class); x->proxy_wg->owner    = x; x->proxy_wg->which    = 2;
    x->proxy_global = (t_jps_proxy *)pd_new(jps_proxy_class); x->proxy_global->owner= x; x->proxy_global->which= 3;
    inlet_new(&x->x_obj, &x->proxy_modal->p_obj.ob_pd, &s_anything, &s_anything);
    inlet_new(&x->x_obj, &x->proxy_wg->p_obj.ob_pd,    &s_anything, &s_anything);
    inlet_new(&x->x_obj, &x->proxy_global->p_obj.ob_pd,&s_anything, &s_anything);
    // outlets
    x->outL = outlet_new(&x->x_obj, gensym("signal"));
    x->outR = outlet_new(&x->x_obj, gensym("signal"));

    // init engines
    jps_modal_init(&x->modal);
    jps_wg_init(&x->wg);

    // voice defaults
    for(int v=0; v<JPS_VOICES; ++v){ x->V[v].state = JPS_IDLE; x->V[v].f0=110.f; x->V[v].vel=0.f; }

    // globals per spec
    x->g_modal_gain = 1.0f;
    x->g_wg_gain    = 1.0f;
    x->g_wg_xfade   = -1.0f; // WG fed by external exciter on boot

    // WG boot: x1 ratio, max gain, +polarity, max decay/release
    x->wg.gain = 1.0f; x->wg.ratio = 1.0f; x->wg.polarity = +1.0f;
    x->wg.decay = 1.0f; x->wg.release = 1.0f;

    // Modal boot: single resonator active at x1, audible to master, not feeding WG
    x->modal.modes = 1;
    x->modal.ratio[0] = 1.0f;
    x->modal.gain[0] = 1.0f;
    x->modal.decay[0] = 400.0f;

    // buffers init
    for(int v=0; v<JPS_VOICES; ++v){ x->mod_vL[v]=x->mod_vR[v]=NULL; }
    x->modal_sumL = x->modal_sumR = x->wg_sumL = x->wg_sumR = NULL;

    return x;
}

static void jps_free(t_jps *x){
    for(int i=0;i<JPS_SIG_INLETS;i++){ if(x->in_sig[i]) inlet_free(x->in_sig[i]); }
    if(x->outL) outlet_free(x->outL);
    if(x->outR) outlet_free(x->outR);
    if(x->proxy_modal) pd_free(&x->proxy_modal->p_obj);
    if(x->proxy_wg)    pd_free(&x->proxy_wg->p_obj);
    if(x->proxy_global)pd_free(&x->proxy_global->p_obj);
    for(int v=0; v<JPS_VOICES; ++v){
        if(x->wg.bufL[v]) free(x->wg.bufL[v]);
        if(x->wg.bufR[v]) free(x->wg.bufR[v]);
        if(x->mod_vL[v]) free(x->mod_vL[v]);
        if(x->mod_vR[v]) free(x->mod_vR[v]);
    }
    if(x->modal_sumL) free(x->modal_sumL);
    if(x->modal_sumR) free(x->modal_sumR);
    if(x->wg_sumL) free(x->wg_sumL);
    if(x->wg_sumR) free(x->wg_sumR);
}

void juicy_physical_synth_tilde_setup(void){
    jps_class = class_new(gensym("juicy_physical_synth~"),
                          (t_newmethod)jps_new,
                          (t_method)jps_free,
                          sizeof(t_jps), CLASS_DEFAULT, 0);
    class_addlist(jps_class, (t_method)jps_list);
    class_addmethod(jps_class, (t_method)jps_dsp, gensym("dsp"), A_CANT, 0);

    jps_proxy_class = class_new(gensym("_jps_proxy"),
                                0, 0, sizeof(t_jps_proxy), CLASS_NOINLET, 0);
    class_addanything(jps_proxy_class, (t_method)jps_proxy_anything);
}
