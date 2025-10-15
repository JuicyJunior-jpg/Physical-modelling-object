// juicy_physical_synth_tilde.c (HOTFIX)
// Stability-focused update: removes risky code paths and adds hard guards.
//  - Stable AR env
//  - Non-null buffers from new()
//  - Defensive allocation in dsp() and perform()
//  - NaN/Inf clamps
//  - Safe delay sizing and filter coeffs
#include "m_pd.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define JPS_VOICES 4
#define JPS_SIG_INLETS (JPS_VOICES*2)
#define JPS_MODAL_MAXMODES 64
#define JPS_MIN_BLOCK 16

static inline float jps_clamp(float x, float lo, float hi){ return (x<lo?lo:(x>hi?hi:x)); }
static inline float jps_clamp01(float v){ return v<0.f?0.f:(v>1.f?1.f:v); }
static inline float jps_midi_to_hz(float n){ return 440.f * powf(2.f, (n-69.f)/12.f); }
static inline float jps_cospow_w(float t){ t=jps_clamp01(t); return sqrtf(1.f - t); }
static inline float jps_cospow_u(float t){ t=jps_clamp01(t); return sqrtf(t); }
static inline float jps_safenan(float x){ return (isfinite(x)? x : 0.f); }

typedef enum { JPS_IDLE=0, JPS_HELD=1, JPS_RELEASE=2 } jps_vstate;
typedef struct { jps_vstate state; float f0, vel; } jps_voice_t;

typedef struct { float z; } jps_onepole;
static inline float onepole_a_from_hz(float hz, float sr){
    if(hz<=0.f) return 0.0f;
    float a = expf(-2.f*M_PI * (hz/sr));
    if(!(a>=0.f && a<1.f)) a = 0.0f;
    return a;
}
static inline float onepole_lp(float x, jps_onepole* st, float a){
    float y = (1.f-a)*x + a*st->z; st->z = y; return y;
}
static inline float onepole_hp(float x, jps_onepole* st, float a){
    float y = x - onepole_lp(x, st, a); return y;
}

typedef struct { float z; } jps_allpass;
static inline float allpass_tick(float x, jps_allpass* st, float g){
    g = jps_clamp(g, -0.98f, 0.98f);
    float y = -g*x + st->z;
    st->z = x + g*y;
    return y;
}

static inline float waveshape(float x, float drive, float symmetry){
    float bias = jps_clamp(symmetry, -1.f, 1.f) * 0.9f;
    x = jps_safenan(x + bias);
    drive = jps_clamp01(drive);
    float y;
    if(drive <= 0.5f){
        float t = drive/0.5f;
        float soft = tanhf(x);
        y = soft*t + (1.f-t)*x;
    }else{
        float t = (drive-0.5f)/0.5f;
        float hard = (x>1.f?1.f:(x<-1.f?-1.f:x));
        float soft = tanhf(x);
        y = hard*t + (1.f-t)*soft;
    }
    y -= bias * 0.5f;
    return jps_safenan(y);
}

// ===== MODAL =====
typedef struct {
    float damper, brightness, position, density, dispersion, anisotropy, contact;
    int   index;
    float ratio[JPS_MODAL_MAXMODES], gain[JPS_MODAL_MAXMODES], attack[JPS_MODAL_MAXMODES], decay[JPS_MODAL_MAXMODES];
    float curve[JPS_MODAL_MAXMODES], pan[JPS_MODAL_MAXMODES], keytrack[JPS_MODAL_MAXMODES];
    int   modes;
    float env[JPS_VOICES][JPS_MODAL_MAXMODES];
    float ph[JPS_VOICES][JPS_MODAL_MAXMODES];
    int   gate[JPS_VOICES];
} jps_modal;

typedef struct {
    int   index;
    float gain, ratio, keytrack, polarity, decay, release, HP, LP;
    float dispersion, position, coupling, push, drive, symmetry;
    float msd_AMT, msd_Freq, msd_Q, msd_INDEX, msd_TILT, msd_Enable, msd_EM;
    // simple delay per voice/channel
    float **bufL, **bufR;
    int   *size, *wpos;
    jps_onepole *hpL, *hpR, *lpL, *lpR;
    jps_allpass *apL, *apR;
} jps_wg;

typedef struct _jps {
    t_object  x_obj;
    t_inlet  *in_sig[JPS_SIG_INLETS];
    t_outlet *outL, *outR;
    struct _jps_proxy *proxy_modal, *proxy_wg, *proxy_global;

    int sr, bs;
    jps_voice_t V[JPS_VOICES];
    jps_modal modal;
    jps_wg    wg;

    float g_modal_gain, g_wg_gain, g_wg_xfade;

    // per-voice modal feed & sums
    float **mod_vL, **mod_vR;
    float *modal_sumL, *modal_sumR;
    float *wg_sumL, *wg_sumR;

    t_sample *excL[JPS_VOICES], *excR[JPS_VOICES];
} t_jps;

typedef struct _jps_proxy { t_object p_obj; t_jps *owner; int which; } t_jps_proxy;
static t_class *jps_class;
static t_class *jps_proxy_class;

static void modal_init(jps_modal* m){
    m->damper=0.5f; m->brightness=0.5f; m->position=0.f; m->density=0.f; m->dispersion=0.f; m->anisotropy=0.f; m->contact=0.f;
    m->index=0; m->modes=1;
    for(int i=0;i<JPS_MODAL_MAXMODES;i++){
        m->ratio[i]=(i==0)?1.f:(float)(i+1);
        m->gain[i]=(i==0)?1.f:(1.f/(1.f+i));
        m->attack[i]=5.f; m->decay[i]=400.f;
        m->curve[i]=0.f; m->pan[i]=0.f; m->keytrack[i]=1.f;
    }
    memset(m->env,0,sizeof(m->env));
    memset(m->ph,0,sizeof(m->ph));
    memset(m->gate,0,sizeof(m->gate));
}
static void wg_alloc_structs(jps_wg* w){
    w->bufL = (float**)calloc(JPS_VOICES,sizeof(float*));
    w->bufR = (float**)calloc(JPS_VOICES,sizeof(float*));
    w->size = (int*)calloc(JPS_VOICES,sizeof(int));
    w->wpos = (int*)calloc(JPS_VOICES,sizeof(int));
    w->hpL = (jps_onepole*)calloc(JPS_VOICES,sizeof(jps_onepole));
    w->hpR = (jps_onepole*)calloc(JPS_VOICES,sizeof(jps_onepole));
    w->lpL = (jps_onepole*)calloc(JPS_VOICES,sizeof(jps_onepole));
    w->lpR = (jps_onepole*)calloc(JPS_VOICES,sizeof(jps_onepole));
    w->apL = (jps_allpass*)calloc(JPS_VOICES,sizeof(jps_allpass));
    w->apR = (jps_allpass*)calloc(JPS_VOICES,sizeof(jps_allpass));
}
static void wg_init(jps_wg* w){
    w->index=0; w->gain=1.f; w->ratio=1.f; w->keytrack=1.f; w->polarity=+1.f;
    w->decay=1.f; w->release=1.f; w->HP=0.f; w->LP=0.f;
    w->dispersion=0.f; w->position=0.f; w->coupling=0.f; w->push=0.f; w->drive=0.f; w->symmetry=0.f;
    w->msd_AMT=0.f; w->msd_Freq=0.f; w->msd_Q=1.f; w->msd_INDEX=0.f; w->msd_TILT=0.f; w->msd_Enable=0.f; w->msd_EM=0.f;
    wg_alloc_structs(w);
}

static int get1f(int argc, t_atom* argv, float* out){
    if(argc<1 || argv[0].a_type!=A_FLOAT) return 0;
    *out = atom_getfloat(argv); return 1;
}

static void jps_list(t_jps* x, t_symbol* s, int argc, t_atom* argv){
    (void)s;
    if(argc<3) return;
    if(argv[0].a_type!=A_FLOAT || argv[1].a_type!=A_FLOAT || argv[2].a_type!=A_FLOAT) return;
    int vix = (int)atom_getfloat(argv) - 1;
    float midi = atom_getfloat(argv+1);
    float vel  = atom_getfloat(argv+2);
    if(vix<0 || vix>=JPS_VOICES) return;
    if(vel>0.f){
        x->V[vix].state=JPS_HELD;
        x->V[vix].f0=jps_midi_to_hz(midi);
        x->V[vix].vel=jps_clamp01(vel);
        x->modal.gate[vix]=1;
        for(int k=0;k<x->modal.modes;k++){ x->modal.env[vix][k]=0.f; }
    }else{
        x->V[vix].state=JPS_RELEASE;
        x->modal.gate[vix]=0;
    }
}
static void jps_modal_msg(t_jps* x, t_symbol* sel, int argc, t_atom* argv){
    float v;
    #define SET(sym,field) if(sel==gensym(sym) && get1f(argc,argv,&v)){ x->modal.field=v; return; }
    SET("damper",damper) SET("brightness",brightness) SET("position",position) SET("density",density)
    SET("dispersion",dispersion) SET("anisotropy",anisotropy) SET("contact",contact)
    if(sel==gensym("index") && get1f(argc,argv,&v)){ int i=(int)v; if(i<0)i=0; if(i>=JPS_MODAL_MAXMODES)i=JPS_MODAL_MAXMODES-1; x->modal.index=i; return; }
    if(sel==gensym("ratio")   && get1f(argc,argv,&v)){ x->modal.ratio[x->modal.index]=v; return; }
    if(sel==gensym("gain")    && get1f(argc,argv,&v)){ x->modal.gain[x->modal.index]=v; return; }
    if(sel==gensym("attack")  && get1f(argc,argv,&v)){ x->modal.attack[x->modal.index]=v; return; }
    if(sel==gensym("decay")   && get1f(argc,argv,&v)){ x->modal.decay[x->modal.index]=v; return; }
    if(sel==gensym("curve")   && get1f(argc,argv,&v)){ x->modal.curve[x->modal.index]=v; return; }
    if(sel==gensym("pan")     && get1f(argc,argv,&v)){ x->modal.pan[x->modal.index]=v; return; }
    if(sel==gensym("keytrack")&& get1f(argc,argv,&v)){ x->modal.keytrack[x->modal.index]=v; return; }
    #undef SET
}
static void jps_wg_msg(t_jps* x, t_symbol* sel, int argc, t_atom* argv){
    float v;
    #define SET(sym,field) if(sel==gensym(sym) && get1f(argc,argv,&v)){ x->wg.field=v; return; }
    SET("index",index) SET("gain",gain) SET("ratio",ratio) SET("keytrack",keytrack) SET("polarity",polarity)
    SET("decay",decay) SET("release",release) SET("HP",HP) SET("LP",LP) SET("dispersion",dispersion)
    SET("position",position) SET("coupling",coupling) SET("push",push) SET("drive",drive) SET("symmetry",symmetry)
    SET("AMT",msd_AMT) SET("Freq",msd_Freq) SET("Q",msd_Q) SET("INDEX",msd_INDEX) SET("TILT",msd_TILT) SET("Enable",msd_Enable) SET("E/M",msd_EM)
    #undef SET
}
static void jps_global_msg(t_jps* x, t_symbol* sel, int argc, t_atom* argv){
    float v;
    if(sel==gensym("Modal_gain") && get1f(argc,argv,&v)){ x->g_modal_gain=v; return; }
    if(sel==gensym("Waveguide_gain") && get1f(argc,argv,&v)){ x->g_wg_gain=v; return; }
    if(sel==gensym("Waveguide_excitation_crossfader") && get1f(argc,argv,&v)){ x->g_wg_xfade=jps_clamp(v,-1.f,1.f); return; }
}

static void modal_render(t_jps* x, int n){
    // sums
    memset(x->modal_sumL, 0, n*sizeof(float));
    memset(x->modal_sumR, 0, n*sizeof(float));
    // per-voice feeds
    for(int v=0; v<JPS_VOICES; ++v){
        memset(x->mod_vL[v], 0, n*sizeof(float));
        memset(x->mod_vR[v], 0, n*sizeof(float));
    }
    jps_modal* m=&x->modal;
    int modes = m->modes; if(modes<1)modes=1; if(modes>JPS_MODAL_MAXMODES)modes=JPS_MODAL_MAXMODES;
    float sr=(float)x->sr;

    for(int v=0; v<JPS_VOICES; ++v){
        float base = fmaxf(20.f, x->V[v].f0);
        float vel  = jps_clamp01(x->V[v].vel);
        int gate   = (x->V[v].state==JPS_HELD);
        for(int k=0;k<modes;k++){
            float freq = base * fmaxf(0.0001f, m->ratio[k]);
            float g    = jps_clamp01(m->gain[k]);
            float pan  = jps_clamp(m->pan[k], -1.f, 1.f);
            float atkS = fmaxf(1.f, m->attack[k]*0.001f*sr);
            float decS = fmaxf(1.f, m->decay[k]*0.001f*sr) * (1.f + 2.f*jps_clamp01(m->damper));
            float a    = gate? expf(-1.f/atkS) : expf(-1.f/decS);
            for(int i=0;i<n;i++){
                // env
                m->env[v][k] = (1.f-a)*(gate?1.f:0.f) + a*m->env[v][k];
                // osc
                m->ph[v][k] += freq/sr; if(m->ph[v][k]>=1.f) m->ph[v][k]-=1.f;
                float s = sinf(2.f*M_PI*m->ph[v][k]) * m->env[v][k] * g * vel;
                // brightness tilt
                float tilt = 1.f + 0.5f*jps_clamp01(m->brightness) * ((float)k/(float)modes);
                s *= tilt;
                float L = s * (0.5f*(1.f - pan));
                float R = s * (0.5f*(1.f + pan));
                x->mod_vL[v][i] += L;
                x->mod_vR[v][i] += R;
            }
        }
        for(int i=0;i<n;i++){ x->modal_sumL[i]+=x->mod_vL[v][i]; x->modal_sumR[i]+=x->mod_vR[v][i]; }
    }
}

static void wg_prepare_voice(t_jps* x, int v){
    float sr = (float)x->sr;
    float f0 = fmaxf(30.f, x->V[v].f0);
    int need = (int)fmaxf(JPS_MIN_BLOCK, sr/f0 * fmaxf(0.5f, x->wg.ratio));
    if(need< JPS_MIN_BLOCK) need = JPS_MIN_BLOCK;
    if(x->wg.size[v] != need || x->wg.bufL[v]==NULL || x->wg.bufR[v]==NULL){
        free(x->wg.bufL[v]); free(x->wg.bufR[v]);
        x->wg.bufL[v]=(float*)calloc(need, sizeof(float));
        x->wg.bufR[v]=(float*)calloc(need, sizeof(float));
        x->wg.size[v]=need;
        x->wg.wpos[v]=0;
        x->wg.hpL[v].z=x->wg.hpR[v].z=x->wg.lpL[v].z=x->wg.lpR[v].z=0.f;
        x->wg.apL[v].z=x->wg.apR[v].z=0.f;
    }
}

static void wg_render(t_jps* x, int n){
    memset(x->wg_sumL,0,n*sizeof(float));
    memset(x->wg_sumR,0,n*sizeof(float));

    float sr=(float)x->sr;
    float a_hp = onepole_a_from_hz(jps_clamp01(x->wg.HP)*sr*0.5f, sr);
    float a_lp = onepole_a_from_hz(jps_clamp01(x->wg.LP)*sr*0.5f, sr);
    float disp = jps_clamp(x->wg.dispersion, -0.95f, 0.95f);
    float pol  = (x->wg.polarity>=0.f)?+1.f:-1.f;
    float fb   = 0.5f + 0.5f*jps_clamp01(x->wg.decay);   // 0.5..1.0
    float rels = 0.5f + 0.5f*jps_clamp01(x->wg.release); // 0.5..1.0

    float t = 0.5f*(x->g_wg_xfade+1.f);
    float w_ext = jps_cospow_w(t);
    float w_mod = jps_cospow_u(t);

    for(int v=0; v<JPS_VOICES; ++v){
        wg_prepare_voice(x,v);
        int sz = x->wg.size[v];
        float *bufL=x->wg.bufL[v], *bufR=x->wg.bufR[v];
        int *wp=&x->wg.wpos[v];

        for(int i=0;i<n;i++){
            float extL = (x->V[v].state==JPS_HELD) ? (float)x->excL[v][i] : 0.f;
            float extR = (x->V[v].state==JPS_HELD) ? (float)x->excR[v][i] : 0.f;
            float modL = x->mod_vL[v][i];
            float modR = x->mod_vR[v][i];
            float inL = jps_safenan(w_ext*extL + w_mod*modL);
            float inR = jps_safenan(w_ext*extR + w_mod*modR);

            int rp = (*wp); if(rp<0) rp=0; if(rp>=sz) rp=0;
            float yL = bufL[rp];
            float yR = bufR[rp];

            float shapedL = waveshape(yL + inL*(1.f + 2.f*x->wg.push), x->wg.drive, x->wg.symmetry);
            float shapedR = waveshape(yR + inR*(1.f + 2.f*x->wg.push), x->wg.drive, x->wg.symmetry);

            yL = onepole_hp(shapedL, &x->wg.hpL[v], a_hp);
            yR = onepole_hp(shapedR, &x->wg.hpR[v], a_hp);
            yL = onepole_lp(yL, &x->wg.lpL[v], a_lp);
            yR = onepole_lp(yR, &x->wg.lpR[v], a_lp);

            yL = allpass_tick(yL, &x->wg.apL[v], disp);
            yR = allpass_tick(yR, &x->wg.apR[v], disp);

            float fbL = pol * yL * fb;
            float fbR = pol * yR * fb;
            if(x->V[v].state!=JPS_HELD){ fbL *= rels; fbR *= rels; }

            bufL[*wp] = jps_safenan(fbL);
            bufR[*wp] = jps_safenan(fbR);
            (*wp)++; if(*wp>=sz) *wp=0;

            float p=jps_clamp01(x->wg.position);
            float tapL = yL*(1.f-p) + yR*(p*0.5f);
            float tapR = yR*(1.f-p) + yL*(p*0.5f);

            x->wg_sumL[i] += tapL * x->wg.gain;
            x->wg_sumR[i] += tapR * x->wg.gain;
        }
    }
}

static t_int *jps_perform(t_int *w){
    t_jps *x = (t_jps *)(w[1]);
    int arg=2;
    for(int v=0; v<JPS_VOICES; ++v){ x->excL[v]=(t_sample*)(w[arg++]); x->excR[v]=(t_sample*)(w[arg++]); }
    t_sample *outL=(t_sample*)(w[arg++]);
    t_sample *outR=(t_sample*)(w[arg++]);
    int n=(int)(w[arg++]);

    if(n<JPS_MIN_BLOCK) n=JPS_MIN_BLOCK; // defensive

    // defensive alloc if dsp() wasn't called (host quirk)
    if(!x->modal_sumL || !x->modal_sumR || !x->wg_sumL || !x->wg_sumR){
        x->modal_sumL=(float*)calloc(n,sizeof(float));
        x->modal_sumR=(float*)calloc(n,sizeof(float));
        x->wg_sumL   =(float*)calloc(n,sizeof(float));
        x->wg_sumR   =(float*)calloc(n,sizeof(float));
        for(int v=0; v<JPS_VOICES; ++v){
            x->mod_vL[v]=(float*)calloc(n,sizeof(float));
            x->mod_vR[v]=(float*)calloc(n,sizeof(float));
        }
    }

    modal_render(x, n);
    wg_render(x, n);

    float gM=x->g_modal_gain, gW=x->g_wg_gain;
    for(int i=0;i<n;i++){
        float L = jps_safenan(gM*x->modal_sumL[i] + gW*x->wg_sumL[i]);
        float R = jps_safenan(gM*x->modal_sumR[i] + gW*x->wg_sumR[i]);
        outL[i]=L; outR[i]=R;
    }
    return (w + 2 + JPS_SIG_INLETS + 3);
}

static void jps_dsp(t_jps *x, t_signal **sp){
    x->sr=(int)sp[0]->s_sr;
    int n=sp[0]->s_n; if(n<JPS_MIN_BLOCK) n=JPS_MIN_BLOCK;

    // (re)alloc per-block buffers
    free(x->modal_sumL); free(x->modal_sumR); free(x->wg_sumL); free(x->wg_sumR);
    x->modal_sumL=(float*)calloc(n,sizeof(float));
    x->modal_sumR=(float*)calloc(n,sizeof(float));
    x->wg_sumL   =(float*)calloc(n,sizeof(float));
    x->wg_sumR   =(float*)calloc(n,sizeof(float));
    for(int v=0; v<JPS_VOICES; ++v){
        free(x->mod_vL[v]); free(x->mod_vR[v]);
        x->mod_vL[v]=(float*)calloc(n,sizeof(float));
        x->mod_vR[v]=(float*)calloc(n,sizeof(float));
    }

    int argc = 2 + JPS_SIG_INLETS + 3;
    t_int *vec = (t_int *)getbytes(argc * sizeof(t_int));
    int k=0; vec[k++]=(t_int)x;
    for(int v=0; v<JPS_VOICES; ++v){ vec[k++]=(t_int)sp[2*v]->s_vec; vec[k++]=(t_int)sp[2*v+1]->s_vec; }
    vec[k++]=(t_int)sp[JPS_SIG_INLETS]->s_vec;
    vec[k++]=(t_int)sp[JPS_SIG_INLETS+1]->s_vec;
    vec[k++]=(t_int)n;
    dsp_addv(jps_perform, argc, vec);
    freebytes(vec, argc*sizeof(t_int));
}

static void *jps_new(void){
    t_jps *x=(t_jps*)pd_new(jps_class);
    for(int i=0;i<JPS_SIG_INLETS;i++) inlet_new(&x->x_obj,&x->x_obj.ob_pd,gensym("signal"),gensym("signal"));

    // proxies
    jps_proxy_class = jps_proxy_class; // silence unused warning
    x->proxy_modal  = (t_jps_proxy*)pd_new(jps_proxy_class); x->proxy_modal->owner=x; x->proxy_modal->which=1;
    x->proxy_wg     = (t_jps_proxy*)pd_new(jps_proxy_class); x->proxy_wg->owner=x; x->proxy_wg->which=2;
    x->proxy_global = (t_jps_proxy*)pd_new(jps_proxy_class); x->proxy_global->owner=x; x->proxy_global->which=3;
    inlet_new(&x->x_obj,&x->proxy_modal->p_obj.ob_pd,&s_anything,&s_anything);
    inlet_new(&x->x_obj,&x->proxy_wg->p_obj.ob_pd,&s_anything,&s_anything);
    inlet_new(&x->x_obj,&x->proxy_global->p_obj.ob_pd,&s_anything,&s_anything);

    x->outL=outlet_new(&x->x_obj,gensym("signal"));
    x->outR=outlet_new(&x->x_obj,gensym("signal"));

    for(int v=0; v<JPS_VOICES; ++v){ x->V[v].state=JPS_IDLE; x->V[v].f0=110.f; x->V[v].vel=0.f; }

    modal_init(&x->modal);
    wg_init(&x->wg);

    x->g_modal_gain=1.f; x->g_wg_gain=1.f; x->g_wg_xfade=-1.f;

    // non-null small buffers
    int n=JPS_MIN_BLOCK;
    x->modal_sumL=(float*)calloc(n,sizeof(float));
    x->modal_sumR=(float*)calloc(n,sizeof(float));
    x->wg_sumL   =(float*)calloc(n,sizeof(float));
    x->wg_sumR   =(float*)calloc(n,sizeof(float));
    x->mod_vL = (float**)calloc(JPS_VOICES,sizeof(float*));
    x->mod_vR = (float**)calloc(JPS_VOICES,sizeof(float*));
    for(int v=0; v<JPS_VOICES; ++v){
        x->mod_vL[v]=(float*)calloc(n,sizeof(float));
        x->mod_vR[v]=(float*)calloc(n,sizeof(float));
    }
    return x;
}
static void jps_free(t_jps *x){
    for(int i=0;i<JPS_SIG_INLETS;i++){}
    if(x->outL) outlet_free(x->outL);
    if(x->outR) outlet_free(x->outR);
    if(x->proxy_modal) pd_free(&x->proxy_modal->p_obj);
    if(x->proxy_wg)    pd_free(&x->proxy_wg->p_obj);
    if(x->proxy_global)pd_free(&x->proxy_global->p_obj);
    free(x->modal_sumL); free(x->modal_sumR); free(x->wg_sumL); free(x->wg_sumR);
    for(int v=0; v<JPS_VOICES; ++v){
        free(x->mod_vL[v]); free(x->mod_vR[v]);
        free(x->wg.bufL[v]); free(x->wg.bufR[v]);
    }
    free(x->mod_vL); free(x->mod_vR);
    free(x->wg.bufL); free(x->wg.bufR);
    free(x->wg.size); free(x->wg.wpos);
    free(x->wg.hpL); free(x->wg.hpR); free(x->wg.lpL); free(x->wg.lpR);
    free(x->wg.apL); free(x->wg.apR);
}

static void jps_proxy_anything(t_jps_proxy* p, t_symbol* s, int argc, t_atom* argv){
    t_jps *x=p->owner;
    if(p->which==1){ jps_modal_msg(x,s,argc,argv); return; }
    if(p->which==2){ jps_wg_msg(x,s,argc,argv); return; }
    if(p->which==3){ jps_global_msg(x,s,argc,argv); return; }
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
