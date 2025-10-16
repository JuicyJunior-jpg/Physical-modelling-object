
// juicy_physical_synth~.c
// “Ultimate” wrapper that embeds juicy_bank~ (modal) + wg_bank~ (waveguide)
// and mixes/routs them inside ONE DSP perform loop.
// Design note: we INCLUDE the original .c files so their static helpers/types
// are available. We DO NOT call their Pd setup() or new() methods. Instead we
// allocate their state structs directly and drive their static perform()
// functions with our own input/output vectors.
//
// IMPORTANT: Keep the two source files next to this one with the exact names:
//   - "juicy_bank_tilde (pre-connection).c"
//   - "wg_bank_tilde (pre-connection).c"
//
// Build targets are provided in the Makefile.
//
// Inlets/Outlets (signals):
//   ~ 8 signal inlets: exciter audio per voice (v1_L, v1_R, … v4_L, v4_R).
//   ~ 2 signal outlets: L/R mix of Modal + Waveguide engines.
//
// Message inlets (float proxies):
//   inlet #9  (float): Modal params (damper/brightness/…/keytrack) — send as "symbol float".
//   inlet #10 (float): Waveguide params (index/gain/…/symmetry + MSD block).
//   inlet #11 (float): Global params (Modal_gain/Waveguide_gain/Waveguide_excitation_crossfader).
//   inlet #12 (list) : Poly voice list exactly like wg_bank~/juicy_bank~ expect (midi/vel, etc.).
//
// Crossfader: -1..1 (constant-power). -1 = pure external exciter → WG, +1 = modal stereo → WG.
// Per-voice gating: WG only ingests when that voice is active (per modal’s voice state).
//
// SAFETY: This file never registers the juicy_bank~ or wg_bank~ Pd classes, so it
// won’t conflict with separate externals you might load. We only use their internals.
//
// © You + Juicy 2025. MIT-ish vibes.

#include "m_pd.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ---- prevent registering the embedded classes ----
#define juicy_bank_tilde_setup juicy_bank_tilde_setup__embedded__do_not_call
#define wg_bank_tilde_setup    wg_bank_tilde_setup__embedded__do_not_call

// Include engines (as provided). Paths with spaces are OK when in quotes.
#include "juicy_bank_tilde (pre-connection).c"
#include "wg_bank_tilde (pre-connection).c"

// ===============================
// Wrapper object
// ===============================

typedef struct _jps {
    t_object  x_obj;
    t_outlet *outL, *outR;

    // Embedded engine states
    t_juicy_bank_tilde modal;
    t_wg_bank          wg;

    // Globals
    t_float modal_gain;   // 0..1
    t_float wg_gain;      // 0..1
    t_float cf;           // -1..1 crossfade (ext exciter ↔ modal stereo)

    // scratch buffers reused per block
    int      bs_cached;
    t_float *zilch;       // zero vector (for unused inputs)
    t_float *modalL, *modalR;
    t_float *wgL, *wgR;
    // per-voice crossfade inputs for WG
    t_float *sL[WG_VOICES*2]; // we’ll reuse pointers into exciter inputs
    t_float *sR[WG_VOICES*2];

    // proxies to route messages
    t_symbol *which_modal;
    t_symbol *which_wg;
    t_symbol *which_glob;
} t_jps;

static t_class *jps_class;

// -------------------------------
// Small helpers
// -------------------------------

static inline void *jps_alloc_zero(size_t count){
    t_float *p = (t_float *)getbytes(count * sizeof(t_float));
    if(p) memset(p, 0, count*sizeof(t_float));
    return p;
}

static inline void jps_ensure_buffers(t_jps *x, int bs){
    if (x->bs_cached == bs && x->zilch) return;
    if (x->zilch){
        freebytes(x->zilch, x->bs_cached*sizeof(t_float));
        freebytes(x->modalL, x->bs_cached*sizeof(t_float));
        freebytes(x->modalR, x->bs_cached*sizeof(t_float));
        freebytes(x->wgL,    x->bs_cached*sizeof(t_float));
        freebytes(x->wgR,    x->bs_cached*sizeof(t_float));
    }
    x->zilch  = (t_float*)jps_alloc_zero(bs);
    x->modalL = (t_float*)jps_alloc_zero(bs);
    x->modalR = (t_float*)jps_alloc_zero(bs);
    x->wgL    = (t_float*)jps_alloc_zero(bs);
    x->wgR    = (t_float*)jps_alloc_zero(bs);
    x->bs_cached = bs;
}

// Constant-power crossfade gains from c ∈ [-1,1] (A = ext, B = modal)
static inline void jps_cf_gains(float c, float *gA, float *gB){
    float p = 0.5f * (c + 1.f);              // map to [0,1]
    float a = cosf((float)M_PI_2 * p);
    float b = sinf((float)M_PI_2 * p);
    *gA = a; *gB = b;
}

// -------------------------------
// Parameter routing (subset/aliases)
// -------------------------------

// Modal body params
static void jps_modal_param(t_jps *x, t_symbol *s, t_float f){
    // aliases: "damper" → damping, "decay" → decya
    if (s==gensym("damper") || s==gensym("damping"))        { juicy_bank_tilde_damping(&x->modal, f); return; }
    if (s==gensym("brightness"))                            { juicy_bank_tilde_brightness(&x->modal, f); return; }
    if (s==gensym("position"))                              { juicy_bank_tilde_position(&x->modal, f); return; }
    if (s==gensym("density"))                               { juicy_bank_tilde_density(&x->modal, f); return; }
    if (s==gensym("dispersion"))                            { juicy_bank_tilde_dispersion(&x->modal, f); return; }
    if (s==gensym("anisotropy") || s==gensym("aniso"))      { juicy_bank_tilde_anisotropy(&x->modal, f); return; }
    if (s==gensym("contact"))                               { juicy_bank_tilde_contact(&x->modal, f); return; }

    if (s==gensym("index"))                                 { juicy_bank_tilde_index(&x->modal, f); return; }
    if (s==gensym("ratio"))                                 { juicy_bank_tilde_ratio(&x->modal, f); return; }
    if (s==gensym("gain"))                                  { juicy_bank_tilde_gain(&x->modal, f); return; }
    if (s==gensym("attack"))                                { juicy_bank_tilde_attack(&x->modal, f); return; }
    if (s==gensym("decay") || s==gensym("decya"))           { juicy_bank_tilde_decya(&x->modal, f); return; }
    if (s==gensym("curve"))                                 { juicy_bank_tilde_curve(&x->modal, f); return; }
    if (s==gensym("pan"))                                   { juicy_bank_tilde_pan(&x->modal, f); return; }
    if (s==gensym("keytrack"))                              { juicy_bank_tilde_keytrack(&x->modal, f); return; }
}

// Waveguide params (string topology only)
static void jps_wg_param(t_jps *x, t_symbol *s, t_float f){
    if (s==gensym("index"))     { x->wg.v_index = x->wg.index  = f; return; }
    if (s==gensym("gain"))      { x->wg.v_gain  = x->wg.gain   = f; return; }
    if (s==gensym("ratio"))     { x->wg.v_ratio = x->wg.ratio  = f; return; }
    if (s==gensym("keytrack"))  { /* handled via proxies in wg normally; noop here */ return; }
    if (s==gensym("polarity"))  { /* plus(>0)/minus(<=0) */ for(int v=0; v<VOICES; ++v){ x->wg.V[v].L[x->wg.active_idx].polarity_plus = (f>0.f)?1:0; } return; }

    if (s==gensym("decay"))     { x->wg.v_decay   = x->wg.decay   = f; return; }
    if (s==gensym("release"))   { x->wg.v_release = x->wg.release = f; return; }
    if (s==gensym("HP"))        { x->wg.v_hp      = x->wg.hp      = f; return; }
    if (s==gensym("LP"))        { x->wg.v_lp      = x->wg.lp      = f; return; }
    if (s==gensym("dispersion")){ x->wg.v_disp    = x->wg.disp    = f; return; }
    if (s==gensym("position"))  { x->wg.v_pos     = x->wg.pos     = f; return; }
    if (s==gensym("coupling"))  { x->wg.v_coupling= x->wg.coupling= f; return; }
    if (s==gensym("push"))      { x->wg.v_push    = x->wg.push    = f; return; }
    if (s==gensym("drive"))     { x->wg.v_drive   = x->wg.drive   = f; return; }
    if (s==gensym("symmetry"))  { x->wg.v_bias    = x->wg.bias    = f; return; }

    // MSD
    if (s==gensym("AMT"))       { x->wg.v_msd_amt  = x->wg.msd_amt  = f; return; }
    if (s==gensym("Freq"))      { x->wg.v_msd_freq = x->wg.msd_freq = f; return; }
    if (s==gensym("Q"))         { x->wg.v_msd_q    = x->wg.msd_q    = f; return; }
    if (s==gensym("INDEX"))     { x->wg.v_msd_pos  = x->wg.msd_pos  = f; return; }
    if (s==gensym("TILT"))      { x->wg.v_edge     = x->wg.edge     = f; return; }
    if (s==gensym("Enable"))    { x->wg.v_msd_enable = x->wg.msd_enable = (f>0.f)?1:0; return; }
    if (s==gensym("E/M"))       { /* not present in current wg core; ignore gracefully */ return; }
}

// Globals
static void jps_global_param(t_jps *x, t_symbol *s, t_float f){
    if (s==gensym("Modal_gain"))                         { x->modal_gain = f; return; }
    if (s==gensym("Waveguide_gain"))                     { x->wg_gain    = f; return; }
    if (s==gensym("Waveguide_excitation_crossfader"))    { x->cf = f;    return; }
}

// -------------------------------
// Voice list (shared to both engines)
// Expect exact grammar those banks use; we forward to their helpers.
// -------------------------------

static void jps_list(t_jps *x, t_symbol *s, int argc, t_atom *argv){
    (void)s;
    // We try a few common grammars the banks expose.
    if (argc==2){ // midi, vel
        juicy_bank_tilde_note_midi(&x->modal, atom_getfloat(argv+0), atom_getfloat(argv+1));
        // WG voice assign: push to voice 0..3 in round-robin by their internal jb/jw helper; if not present, we set active_idx=0 and update that voice:
        // Minimal: set f0 via modal's v[].f0 so WG reads the same pitch from ratio math; we keep WG purely exciter-driven here.
    } else if (argc==3){ // voice, midi, vel
        int v = (int)atom_getfloat(argv+0); if (v<0) v=0; if (v>=VOICES) v=VOICES-1;
        float midi = atom_getfloat(argv+1), vel = atom_getfloat(argv+2);
        // Modal per-voice on/off via jb_note_on with f0 from midi and set target voice
        jb_note_on_voice(&x->modal, v, jb_midi_to_hz(midi), vel);
        // Mirror minimal state into WG voice energy gate
        x->wg.V[v].active = (vel>0.f)?1:0;
    }
}

// -------------------------------
// Perform
// -------------------------------

static t_int *jps_perform(t_int *w){
    t_jps   *x  = (t_jps *)(w[1]);
    // exciter per-voice per-side inlets
    t_float *v1L = (t_float *)(w[2]);
    t_float *v1R = (t_float *)(w[3]);
    t_float *v2L = (t_float *)(w[4]);
    t_float *v2R = (t_float *)(w[5]);
    t_float *v3L = (t_float *)(w[6]);
    t_float *v3R = (t_float *)(w[7]);
    t_float *v4L = (t_float *)(w[8]);
    t_float *v4R = (t_float *)(w[9]);
    t_float *outL= (t_float *)(w[10]);
    t_float *outR= (t_float *)(w[11]);
    int       n  = (int)(w[12]);

    // 1) Ensure buffers
    jps_ensure_buffers(x, n);

    // 2) Run MODAL engine (inputs: zero main inL/inR, plus 8 per-voice exciter lanes)
    t_int argvM[2 + 12 + 1];
    int a=0;
    argvM[a++] = (t_int)(&x->modal);
    argvM[a++] = (t_int)(x->zilch);
    argvM[a++] = (t_int)(x->zilch);
    argvM[a++] = (t_int)(v1L);
    argvM[a++] = (t_int)(v1R);
    argvM[a++] = (t_int)(v2L);
    argvM[a++] = (t_int)(v2R);
    argvM[a++] = (t_int)(v3L);
    argvM[a++] = (t_int)(v3R);
    argvM[a++] = (t_int)(v4L);
    argvM[a++] = (t_int)(v4R);
    argvM[a++] = (t_int)(x->modalL);
    argvM[a++] = (t_int)(x->modalR);
    argvM[a++] = (t_int)(n);
    juicy_bank_tilde_perform(argvM);

    // 3) Build WG inputs by crossfading External Exciter vs Modal stereo
    float gExt, gMod; jps_cf_gains(x->cf, &gExt, &gMod);
    // Quick voice gates from modal state
    int gV[VOICES]; for(int v=0; v<VOICES; ++v){ gV[v] = (x->modal.v[v].state != V_IDLE); }

    // Build 8 input buffers for WG in the expected order
    // sL[0..3] == v1..v4 left, sR[0..3] == v1..v4 right
    t_float *sL_in[VOICES] = { v1L, v2L, v3L, v4L };
    t_float *sR_in[VOICES] = { v1R, v2R, v3R, v4R };

    // temporary stack arrays for mixed inputs (we reuse modalL/R to avoid extra alloc if safe)
    // but to avoid overwriting, allocate local automatic buffers.
    t_float mixL1[n], mixR1[n], mixL2[n], mixR2[n];

    for(int i=0;i<n;i++){
        float mL = x->modalL[i];
        float mR = x->modalR[i];
        // precompute per-voice mixed signals
        mixL1[i] = gExt * v1L[i] + gMod * mL;
        mixR1[i] = gExt * v1R[i] + gMod * mR;
        mixL2[i] = gExt * v2L[i] + gMod * mL;
        mixR2[i] = gExt * v2R[i] + gMod * mR;
        // voice 3/4 done directly later to save a couple lines
    }

    // 4) Run WG engine by invoking its perform() with our eight inputs
    t_float w_v3L[n], w_v3R[n], w_v4L[n], w_v4R[n];
    for(int i=0;i<n;i++){
        float mL = x->modalL[i], mR = x->modalR[i];
        w_v3L[i] = gExt * v3L[i] + gMod * mL;
        w_v3R[i] = gExt * v3R[i] + gMod * mR;
        w_v4L[i] = gExt * v4L[i] + gMod * mL;
        w_v4R[i] = gExt * v4R[i] + gMod * mR;
        // apply simple voice gate
        if(!gV[0]){ mixL1[i]=0.f; mixR1[i]=0.f; }
        if(!gV[1]){ mixL2[i]=0.f; mixR2[i]=0.f; }
        if(!gV[2]){ w_v3L[i]=0.f; w_v3R[i]=0.f; }
        if(!gV[3]){ w_v4L[i]=0.f; w_v4R[i]=0.f; }
    }

    t_int argvW[2 + 12 + 1];
    int b=0;
    argvW[b++] = (t_int)(&x->wg);
    // order: v1L, v1R, v2L, v2R, v3L, v3R, v4L, v4R, outL, outR, n
    argvW[b++] = (t_int)(mixL1);
    argvW[b++] = (t_int)(mixR1);
    argvW[b++] = (t_int)(mixL2);
    argvW[b++] = (t_int)(mixR2);
    argvW[b++] = (t_int)(w_v3L);
    argvW[b++] = (t_int)(w_v3R);
    argvW[b++] = (t_int)(w_v4L);
    argvW[b++] = (t_int)(w_v4R);
    argvW[b++] = (t_int)(x->wgL);
    argvW[b++] = (t_int)(x->wgR);
    argvW[b++] = (t_int)(n);
    wg_bank_perform(argvW);

    // 5) Final mix
    float mg = x->modal_gain, wg = x->wg_gain;
    for(int i=0;i<n;i++){
        outL[i] = wg * x->wgL[i] + mg * x->modalL[i];
        outR[i] = wg * x->wgR[i] + mg * x->modalR[i];
    }
    return w + 13;
}

static void jps_dsp(t_jps *x, t_signal **sp){
    int n = sp[0]->s_n;
    jps_ensure_buffers(x, n);
    x->modal.sr = sp[0]->s_sr;
    x->wg.sr    = sp[0]->s_sr;
    x->wg.bs    = n;
    x->wg.k_smooth = 1.f - expf(-1.f/(0.01f * x->wg.sr)); // ~10ms smoothing

    t_int argv[2 + 10 + 1];
    int a=0;
    argv[a++] = (t_int)x;
    // 8 inputs
    for(int i=0;i<8;i++) argv[a++] = (t_int)(sp[i]->s_vec);
    // 2 outs
    argv[a++] = (t_int)(sp[8]->s_vec);
    argv[a++] = (t_int)(sp[9]->s_vec);
    argv[a++] = (t_int)(n);
    dsp_addv(jps_perform, a, argv);
}

// -------------------------------
// Message plumbing
// -------------------------------

static void jps_anything(t_jps *x, t_symbol *s, int argc, t_atom *argv){
    (void)argc;
    if (s==gensym("Modal_gain") || s==gensym("Waveguide_gain") || s==gensym("Waveguide_excitation_crossfader")){
        if (argc>0) jps_global_param(x, s, atom_getfloat(argv));
        return;
    }
    // Fallback: treat symbol as modal or waveguide param depending on which inlet the float came to.
    // We expose separate inlets via "float" proxies that set x->which_* before calling here.
}

static void jps_float_modal(t_jps *x, t_floatarg f){
    // symbol must arrive first; but Pd sends float only — so we keep last-tag heuristic.
    // Users should send "symbol value" via [pack s f] for reliability.
    if (!x->which_modal) x->which_modal = gensym("gain");
    jps_modal_param(x, x->which_modal, f);
}
static void jps_selector_modal(t_jps *x, t_symbol *s){ x->which_modal = s; }

static void jps_float_wg(t_jps *x, t_floatarg f){
    if (!x->which_wg) x->which_wg = gensym("gain");
    jps_wg_param(x, x->which_wg, f);
}
static void jps_selector_wg(t_jps *x, t_symbol *s){ x->which_wg = s; }

static void jps_float_global(t_jps *x, t_floatarg f){
    if (!x->which_glob) x->which_glob = gensym("Waveguide_excitation_crossfader");
    jps_global_param(x, x->which_glob, f);
}
static void jps_selector_global(t_jps *x, t_symbol *s){ x->which_glob = s; }

// -------------------------------
// New / Free
// -------------------------------


static void *jps_new(void){
    t_jps *x = (t_jps *)pd_new(jps_class);

    x->modal_gain = 0.5f;
    x->wg_gain    = 0.5f;
    x->cf         = -1.f; // start purely external exciter→WG
    x->bs_cached  = 0;
    x->zilch = x->modalL = x->modalR = x->wgL = x->wgR = NULL;
    x->which_modal = x->which_wg = x->which_glob = NULL;

    // Init modal defaults (scratch: one resonator audible at x1, etc.)
    x->modal.sr = sys_getsr(); if (x->modal.sr<=0) x->modal.sr = 48000;
    x->modal.n_modes=20; x->modal.edit_idx=0; x->modal.max_voices = JB_MAX_VOICES;
    for(int i=0;i<JB_MAX_MODES;i++){
        x->modal.base[i].active=(i<20);
        x->modal.base[i].base_ratio=(float)(i+1);
        x->modal.base[i].base_decay_ms=500.f;
        x->modal.base[i].base_gain= (i==0)? 0.5f : 0.0f; // only first resonator audible by default
        x->modal.base[i].attack_ms=0.f;
        x->modal.base[i].curve_amt=0.f;
        x->modal.base[i].pan=(i==0)?0.f:((i&1)?-0.2f:0.2f);
        x->modal.base[i].keytrack=1;
        x->modal.base[i].disp_signature=0.f;
        x->modal.base[i].micro_sig=0.f;
    }
    x->modal.basef0_ref = 261.626f;
    x->modal.damping=0.f; x->modal.brightness=0.5f; x->modal.position=0.f;
    x->modal.density_amt=0.f; x->modal.density_mode=DENSITY_PIVOT;
    x->modal.dispersion=0.f; x->modal.dispersion_last=-1.f;
    x->modal.aniso=0.f; x->modal.aniso_eps=0.02f;
    x->modal.contact_amt=0.f; x->modal.contact_sym=0.f;
    x->modal.phase_rand=1.f; x->modal.phase_debug=0;
    x->modal.bandwidth=0.1f; x->modal.micro_detune=0.1f;
    for(int v=0; v<JB_MAX_VOICES; ++v){
        x->modal.v[v].state=V_IDLE; x->modal.v[v].f0=x->modal.basef0_ref; x->modal.v[v].vel=0.f; x->modal.v[v].energy=0.f;
        for(int i=0;i<JB_MAX_MODES;i++){ x->modal.v[v].disp_offset[i]=x->modal.v[v].disp_target[i]=0.f; x->modal.v[v].cr_gain_mul[i]=x->modal.v[v].cr_decay_mul[i]=1.f; }
    }
    x->modal.hp_a=0.f; x->modal.hpL_x1=x->modal.hpL_y1=x->modal.hpR_x1=x->modal.hpR_y1=0.f;
    x->modal.exciter_mode = 1; // per-voice 8-in

    // Init WG defaults (string, x1 ratio, max gain, plus polarity)
    memset(&x->wg, 0, sizeof(t_wg_bank));
    x->wg.sr = sys_getsr(); if (x->wg.sr<=0) x->wg.sr = 48000;
    x->wg.bs = 64; // will be replaced in dsp()
    x->wg.topo = TOPO_STRING;
    x->wg.v_gain = x->wg.gain = 1.f;
    x->wg.v_ratio = x->wg.ratio = 1.f;
    x->wg.v_index = x->wg.index = 1.f;
    x->wg.v_decay = x->wg.decay = 1.f;
    x->wg.v_release = x->wg.release = 1.f;
    x->wg.v_hp = x->wg.hp = 0.f;
    x->wg.v_lp = x->wg.lp = 1.f;
    x->wg.active_idx = 0;
    for(int v=0; v<VOICES; ++v){
        x->wg.V[v].active = 0;
        for(int l=0; l<WG_LINES; ++l){
            x->wg.V[v].L[l].keytrack_on = 1;
            x->wg.V[v].L[l].polarity_plus = 1;
        }
    }

    // Add the remaining 7 signal inlets (leftmost is implicit via CLASS_MAINSIGNALIN)
    for (int i=0;i<7;i++) inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);

    // Signal outlets
    x->outL = outlet_new(&x->x_obj, &s_signal);
    x->outR = outlet_new(&x->x_obj, &s_signal);

    // Float/message inlets (3 float proxies + one list inlet)
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_symbol, gensym("modal_select"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float,  gensym("modal"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_symbol, gensym("wg_select"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float,  gensym("wg"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_symbol, gensym("global_select"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float,  gensym("global"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_list,   gensym("list")); // poly midi list

    return (void*)x;
}
}

static void jps_free(t_jps *x){
    if (x->zilch){ freebytes(x->zilch,  x->bs_cached*sizeof(t_float)); }
    if (x->modalL){ freebytes(x->modalL, x->bs_cached*sizeof(t_float)); }
    if (x->modalR){ freebytes(x->modalR, x->bs_cached*sizeof(t_float)); }
    if (x->wgL){ freebytes(x->wgL, x->bs_cached*sizeof(t_float)); }
    if (x->wgR){ freebytes(x->wgR, x->bs_cached*sizeof(t_float)); }
}

// -------------------------------
// Setup
// -------------------------------


void juicy_physical_synth_tilde_setup(void){
    jps_class = class_new(gensym("juicy_physical_synth~"),
                          (t_newmethod)jps_new,
                          (t_method)jps_free,
                          sizeof(t_jps),
                          CLASS_DEFAULT, 0);

    class_addmethod(jps_class, (t_method)jps_dsp, gensym("dsp"), A_CANT, 0);
    CLASS_MAINSIGNALIN(jps_class, t_jps, cf); // leftmost signal is implicit; we reuse cf as storage

    // Param selectors (choose which param the next float targets)
    class_addmethod(jps_class, (t_method)jps_selector_modal,  gensym("modal_select"),  A_SYMBOL, 0);
    class_addmethod(jps_class, (t_method)jps_selector_wg,     gensym("wg_select"),     A_SYMBOL, 0);
    class_addmethod(jps_class, (t_method)jps_selector_global, gensym("global_select"), A_SYMBOL, 0);

    // Float messages routed to namespaces
    class_addmethod(jps_class, (t_method)jps_float_modal,  gensym("modal"),  A_DEFFLOAT, 0);
    class_addmethod(jps_class, (t_method)jps_float_wg,     gensym("wg"),     A_DEFFLOAT, 0);
    class_addmethod(jps_class, (t_method)jps_float_global, gensym("global"), A_DEFFLOAT, 0);

    // Poly list
    class_addlist(jps_class, (t_method)jps_list);

    // anything fallback
    class_addanything(jps_class, (t_method)jps_anything);
}

