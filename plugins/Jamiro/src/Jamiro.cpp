// Jamiro - an 8-voice, 3-oscillator virtual-analogue synth for the MPC, by Yann.
//
// SPDX-License-Identifier: MIT
//
// A recreation of the "KS" synth engine of the Novation X-Station (shared with the
// KS4/KS5/KS Rack/K-Station): same layout, same 0..127 / -64..+63 parameter numbers,
// so patches translate one to one. No sounds ship with the plugin: KS/X-Station
// .syx banks dropped into Content/Banks/ are converted at start-up (ks_import.h).
// Not affiliated with or endorsed by Novation.
//
// Signal flow
// -----------
//   per voice (8 voices, unison stacks voices on one note):
//     Osc 1 ─┐  (sync 1>2)       sine/tri/saw(+double saw)/pulse/4 noises/9 tables
//     Osc 2 ─┼─ FM 2>3 ─ Osc 3   pitch: octave, semi, cent, LFO1, mod env, bend, glide
//     Noise ─┤                   PW / double-saw width: position, LFO2, mod env
//     Ring 1*2┘
//       └─> Mixer ─> Filter (LP/BP/HP, 12 or 24 dB, overdrive, Q normalise)
//                     cutoff + mod env + LFO2 + key track + wheel/aftertouch
//              ─> Amp envelope x velocity
//   all voices ─> Mix level ─> Distortion ─> Tilt EQ ─> Pan (+auto-pan)
//              ─> Chorus/Phaser ─> Delay (send) ─> Reverb (send) ─> Volume
//
//   Arpeggiator (internal, follows the MPC tempo): Up, Down, Up/Down 1 & 2,
//   As played, Random, Chord; 1-4 octaves; gate; latch.

#include "mpc_plugin.h"
#include "dsp/math.h"
#include "dsp/svf.h"
#include "dsp/adsr.h"
#include "dsp/midi_cc.h"
#include "dsp/noise.h"
#include "dsp/smoother.h"

#include "jamiro_dsp.h"

using namespace mpc::dsp;
using namespace jamiro;

namespace {

// Parameter order is PERMANENT (saved projects refer to it). Append only.
enum Param
{
    // Voice / global
    kVolume,      // master volume, dB (parameter 0: harmless, the MPC nudges it)
    kMixLevel,    // mixer output level 0..30 dB (pre effects)
    kVoiceMode,   // Mono, Mono AG (auto-glide), Poly 1, Poly 2
    kUnison,      // voices per note 1..8
    kUniDetune,   // spread between unison voices
    kDrift,       // slow random detune per voice ("VCO drift")
    kPorta,       // portamento time
    kBendRange,   // pitch bend semitones
    kSync,        // osc 1 > 2 hard sync

    // Oscillators: 9 parameters each, repeated for osc 1, 2, 3
    kO1Wave, kO1Oct, kO1Semi, kO1Cent, kO1Lfo, kO1Env, kO1Pw, kO1PwLfo, kO1PwEnv,
    kO2Wave, kO2Oct, kO2Semi, kO2Cent, kO2Lfo, kO2Env, kO2Pw, kO2PwLfo, kO2PwEnv,
    kO3Wave, kO3Oct, kO3Semi, kO3Cent, kO3Lfo, kO3Env, kO3Pw, kO3PwLfo, kO3PwEnv,

    // Mixer
    kO1Lvl, kO2Lvl, kO3Lvl, kNoiseLvl, kRingLvl,
    // FM (osc 2 modulates osc 3) with its own AD envelope
    kFmLvl, kFmEnv, kFmAtk, kFmDec,

    // Filter
    kFType, kFSlope, kCutoff, kReso, kDrive, kQNorm, kKeyTrack, kFEnv, kFLfo,

    // Envelopes
    kAAtk, kADec, kASus, kARel, kAVel,
    kMAtk, kMDec, kMSus, kMRel, kMVel,

    // LFOs
    kL1Wave, kL1Speed, kL1Delay, kL1Sync, kL1Key,
    kL2Wave, kL2Speed, kL2Delay, kL2Sync, kL2Key,

    // Mod wheel / aftertouch routings
    kMwVib, kMwFilt, kMwFLfo, kAtVib, kAtFilt,

    // Arpeggiator
    kArpOn, kArpMode, kArpOct, kArpRate, kArpGate, kArpLatch, kArpTempo,

    // Effects
    kDist, kEqLevel, kEqFreq,
    kChSend, kChType, kChRate, kChDepth, kChFb,
    kDlSend, kDlTime, kDlSync, kDlFb, kDlWidth,
    kRvSend, kRvType, kRvDecay,
    kPan, kPanDepth, kPanRate,

    // Added after the first draft (kept at the end so the order never changes)
    kMixL1O1,     // LFO1 -> osc 1 level
    kMixL2O1,     // LFO2 -> osc 1 level
    kMixL2O2,     // LFO2 -> osc 2 level
    kMixAdO3,     // FM (AD) envelope -> osc 3 level
    kFiltVel,     // velocity -> cutoff
    kL1Uni,       // LFO1 unipolar (0..+1 instead of -1..+1)
    kL2Uni,       // LFO2 unipolar

    kNumParams
};

constexpr int kOscStride = kO2Wave - kO1Wave;
enum OscField { fWave, fOct, fSemi, fCent, fLfo, fEnv, fPw, fPwLfo, fPwEnv };

} // namespace

#include "ks_import.h"

class Jamiro : public mpc::Plugin
{
public:
    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo i;
        i.name = "Jamiro";
        i.vendor = "Yann";
        i.uniqueId = MPC_FOURCC('Y', 'A', 'U', 'p');   // == "uid" in plugin.json. Never change.
        i.version = 1000;
        i.isInstrument = true;
        i.numInputs = 0;
        i.numOutputs = 2;
        return i;
    }

    Jamiro() : mpc::Plugin(makeInfo())
    {
        using mpc::floatParam; using mpc::intParam; using mpc::boolParam; using mpc::choiceParam;
        auto u7  = [](const char* id, const char* name, float def) { return floatParam(id, name, 0.0f, 127.0f, def, "", 0); };
        auto bip = [](const char* id, const char* name, float def) { return floatParam(id, name, -64.0f, 63.0f, def, "", 0); };

        addParam(floatParam("vol", "Volume", -40.0f, 6.0f, -6.0f, "dB", 1));
        addParam(floatParam("mixlvl", "Mix Level", 0.0f, 30.0f, 0.0f, "dB", 0));
        addParam(choiceParam("voicemode", "Voice Mode", {"Mono", "Mono AG", "Poly 1", "Poly 2"}, 2));
        addParam(intParam("unison", "Unison", 1, 8, 1));
        addParam(u7("unidet", "Uni Detune", 25.0f));
        addParam(u7("drift", "VCO Drift", 0.0f));
        addParam(u7("porta", "Portamento", 0.0f));
        addParam(intParam("bend", "Bend Range", 0, 12, 2, "st"));
        addParam(boolParam("sync", "Osc Sync", false));

        static const char* kIds[3][9] = {
            {"o1wave", "o1oct", "o1semi", "o1cent", "o1lfo", "o1env", "o1pw", "o1pwlfo", "o1pwenv"},
            {"o2wave", "o2oct", "o2semi", "o2cent", "o2lfo", "o2env", "o2pw", "o2pwlfo", "o2pwenv"},
            {"o3wave", "o3oct", "o3semi", "o3cent", "o3lfo", "o3env", "o3pw", "o3pwlfo", "o3pwenv"} };
        static const char* kNames[3][9] = {
            {"O1 Wave", "O1 Octave", "O1 Semi", "O1 Cent", "O1 LFO1", "O1 ModEnv", "O1 PW", "O1 PW LFO2", "O1 PW Env"},
            {"O2 Wave", "O2 Octave", "O2 Semi", "O2 Cent", "O2 LFO1", "O2 ModEnv", "O2 PW", "O2 PW LFO2", "O2 PW Env"},
            {"O3 Wave", "O3 Octave", "O3 Semi", "O3 Cent", "O3 LFO1", "O3 ModEnv", "O3 PW", "O3 PW LFO2", "O3 PW Env"} };
        for (int o = 0; o < 3; ++o)
        {
            addParam(choiceParam(kIds[o][0], kNames[o][0], oscWaveNames(), kSaw));
            addParam(intParam(kIds[o][1], kNames[o][1], -3, 3, 0, "oct"));   // -3..+3 (KS patches use -1..+2)
            addParam(intParam(kIds[o][2], kNames[o][2], -12, 12, 0, "st"));
            addParam(floatParam(kIds[o][3], kNames[o][3], -50.0f, 50.0f, 0.0f, "ct", 0));
            addParam(bip(kIds[o][4], kNames[o][4], 0.0f));
            addParam(bip(kIds[o][5], kNames[o][5], 0.0f));
            addParam(bip(kIds[o][6], kNames[o][6], 0.0f));
            addParam(bip(kIds[o][7], kNames[o][7], 0.0f));
            addParam(bip(kIds[o][8], kNames[o][8], 0.0f));
        }

        addParam(u7("o1lvl", "Osc1 Level", 127.0f));
        addParam(u7("o2lvl", "Osc2 Level", 0.0f));
        addParam(u7("o3lvl", "Osc3 Level", 0.0f));
        addParam(u7("noiselvl", "Noise Level", 0.0f));
        addParam(u7("ringlvl", "Ring Level", 0.0f));
        addParam(u7("fmlvl", "FM 2>3 Lvl", 0.0f));
        addParam(bip("fmenv", "FM Env Amt", 0.0f));
        addParam(u7("fmatk", "FM Attack", 0.0f));
        addParam(u7("fmdec", "FM Decay", 64.0f));

        addParam(choiceParam("ftype", "Filter Type", {"Low Pass", "Band Pass", "High Pass"}, 0));
        addParam(choiceParam("fslope", "Slope", {"12 dB", "24 dB"}, 1));
        addParam(u7("cutoff", "Cutoff", 127.0f));
        addParam(u7("reso", "Resonance", 0.0f));
        addParam(u7("drive", "Overdrive", 0.0f));
        addParam(u7("qnorm", "Q Normalise", 64.0f));
        addParam(u7("keytrk", "Key Track", 64.0f));
        addParam(bip("fenv", "Filt Env", 0.0f));
        addParam(bip("flfo", "Filt LFO2", 0.0f));

        addParam(u7("aatk", "Amp Attack", 0.0f));
        addParam(u7("adec", "Amp Decay", 64.0f));
        addParam(u7("asus", "Amp Sustain", 127.0f));
        addParam(u7("arel", "Amp Release", 30.0f));
        addParam(bip("avel", "Amp Velo", 0.0f));
        addParam(u7("matk", "Mod Attack", 0.0f));
        addParam(u7("mdec", "Mod Decay", 64.0f));
        addParam(u7("msus", "Mod Sustain", 0.0f));
        addParam(u7("mrel", "Mod Release", 30.0f));
        addParam(bip("mvel", "Mod Velo", 0.0f));

        addParam(choiceParam("l1wave", "LFO1 Wave", lfoWaveNames(), kLTri));
        addParam(u7("l1speed", "LFO1 Speed", 68.0f));
        addParam(u7("l1delay", "LFO1 Delay", 0.0f));
        addParam(choiceParam("l1sync", "LFO1 Sync", lfoSyncNames(), 0));
        addParam(boolParam("l1key", "LFO1 KeySync", false));
        addParam(choiceParam("l2wave", "LFO2 Wave", lfoWaveNames(), kLTri));
        addParam(u7("l2speed", "LFO2 Speed", 68.0f));
        addParam(u7("l2delay", "LFO2 Delay", 0.0f));
        addParam(choiceParam("l2sync", "LFO2 Sync", lfoSyncNames(), 0));
        addParam(boolParam("l2key", "LFO2 KeySync", false));

        addParam(bip("mwvib", "MW Vibrato", 10.0f));
        addParam(bip("mwfilt", "MW Filter", 0.0f));
        addParam(bip("mwflfo", "MW Filt LFO", 0.0f));
        addParam(bip("atvib", "AT Vibrato", 0.0f));
        addParam(bip("atfilt", "AT Filter", 0.0f));

        addParam(boolParam("arpon", "Arp On", false));
        addParam(choiceParam("arpmode", "Arp Mode", {"Up", "Down", "Up-Down 1", "Up-Down 2", "Played", "Random", "Chord"}, 0));
        addParam(intParam("arpoct", "Arp Octaves", 1, 4, 1));
        addParam(choiceParam("arprate", "Arp Rate", arpRateNames(), 3));
        addParam(u7("arpgate", "Arp Gate", 64.0f));
        addParam(boolParam("arplatch", "Arp Latch", false));
        addParam(floatParam("arptempo", "Arp Tempo", 64.0f, 191.0f, 120.0f, "bpm", 0));

        addParam(u7("dist", "Distortion", 0.0f));
        addParam(bip("eqlvl", "EQ Tilt", 0.0f));
        addParam(u7("eqfreq", "EQ Freq", 64.0f));
        addParam(u7("chsend", "Chorus Mix", 0.0f));
        addParam(choiceParam("chtype", "Chorus Type", {"Chorus", "Phaser"}, 0));
        addParam(u7("chrate", "Chorus Rate", 40.0f));
        addParam(u7("chdepth", "Chorus Depth", 64.0f));
        addParam(bip("chfb", "Chorus Fdbk", 0.0f));
        addParam(u7("dlsend", "Delay Send", 0.0f));
        addParam(u7("dltime", "Delay Time", 64.0f));
        addParam(choiceParam("dlsync", "Delay Sync", delaySyncNames(), 0));
        addParam(u7("dlfb", "Delay Fdbk", 40.0f));
        addParam(u7("dlwidth", "Delay Width", 127.0f));
        addParam(u7("rvsend", "Reverb Send", 0.0f));
        addParam(choiceParam("rvtype", "Reverb Type", {"Echo", "Small Room", "Small Hall", "Large Room", "Large Hall", "Great Hall"}, 2));
        addParam(u7("rvdecay", "Reverb Decay", 64.0f));
        addParam(bip("pan", "Pan", 0.0f));
        addParam(u7("pandepth", "AutoPan Dep", 0.0f));
        addParam(u7("panrate", "AutoPan Rate", 40.0f));

        addParam(bip("mixl1o1", "LFO1>O1 Lvl", 0.0f));
        addParam(bip("mixl2o1", "LFO2>O1 Lvl", 0.0f));
        addParam(bip("mixl2o2", "LFO2>O2 Lvl", 0.0f));
        addParam(bip("mixado3", "FMEnv>O3 Lvl", 0.0f));
        addParam(bip("fvel", "Filt Velo", 0.0f));
        addParam(boolParam("l1uni", "LFO1 Unipol", false));
        addParam(boolParam("l2uni", "LFO2 Unipol", false));

        if (numParams() != kNumParams) mpc::log("Jamiro: parameter count mismatch (%d vs %d)", numParams(), (int)kNumParams);

        // Programs: a few original patches, then every KS/X-Station program found in
        // Content/Banks/*.syx (the user supplies those files; none ship with the plugin).
        std::vector<float> defaults((size_t)numParams());
        for (int i = 0; i < numParams(); ++i) defaults[(size_t)i] = param(i).def;
        auto program = [&](const char* name, std::initializer_list<std::pair<int, float>> overrides)
        {
            std::vector<float> v = defaults;
            for (const auto& o : overrides) v[(size_t)o.first] = o.second;
            addProgram(name, std::move(v));
        };
        program("Init Program", {});
        program("Jamiro Saw Lead", {{kVoiceMode, 1}, {kUnison, 3}, {kUniDetune, 30}, {kPorta, 40},
                                    {kO2Lvl, 100}, {kO2Cent, 9}, {kCutoff, 88}, {kReso, 30}, {kFEnv, 20}, {kMDec, 60},
                                    {kARel, 50}, {kChSend, 40}, {kDlSend, 30}, {kDlSync, 9}, {kVolume, -1}});
        program("Jamiro Sub Bass", {{kVoiceMode, 0}, {kO1Wave, kPulse}, {kO2Wave, kSine}, {kO2Oct, -1}, {kO2Lvl, 110},
                                    {kCutoff, 50}, {kReso, 25}, {kFEnv, 30}, {kMDec, 45}, {kADec, 70}, {kASus, 110}, {kARel, 20}, {kVolume, 0}});
        program("Jamiro Warm Pad", {{kUnison, 2}, {kUniDetune, 35}, {kO1Pw, 40}, {kO1PwLfo, 20}, {kO2Lvl, 90}, {kO2Oct, 1}, {kO2Cent, -7},
                                    {kCutoff, 70}, {kFEnv, 12}, {kMAtk, 80}, {kMDec, 100}, {kMSus, 60},
                                    {kAAtk, 75}, {kADec, 90}, {kARel, 95}, {kL2Speed, 40}, {kChSend, 70}, {kRvSend, 60}, {kRvDecay, 90}, {kVolume, -3}});
        program("Jamiro Funk Clav", {{kO1Wave, kFirstTable + 7}, {kCutoff, 62}, {kReso, 55}, {kFEnv, 34}, {kMDec, 40}, {kMVel, 30},
                                     {kADec, 72}, {kASus, 0}, {kARel, 25}, {kAVel, 40}, {kDist, 20}, {kVolume, 2}});
        program("Jamiro Arp Pulse", {{kO1Wave, kPulse}, {kO1PwLfo, 30}, {kCutoff, 60}, {kReso, 70}, {kFEnv, 28}, {kMDec, 50},
                                     {kADec, 60}, {kASus, 0}, {kARel, 30}, {kArpOn, 1}, {kArpMode, 2}, {kArpOct, 2},
                                     {kDlSend, 45}, {kDlSync, 9}, {kDlFb, 55}, {kVolume, 2}});
        program("Jamiro FM Bell", {{kO2Wave, kSine}, {kO3Wave, kSine}, {kO1Lvl, 0}, {kO3Lvl, 120}, {kO2Oct, 1}, {kO2Semi, 7},
                                   {kFmLvl, 30}, {kFmEnv, 40}, {kFmDec, 85}, {kADec, 105}, {kASus, 0}, {kARel, 90}, {kRvSend, 50}, {kVolume, -2}});
        for (ksimport::ImportedProgram& p : ksimport::importAll(defaults))
            addProgram(p.name.c_str(), std::move(p.values));

        // Build lookup tables now, not on the audio thread.
        (void)WaveTables::instance();
        (void)WavetableSine::lookup(0.0f);
    }

    void prepare(double sampleRate, int /*maxBlockSize*/) override
    {
        sr_ = (float)sampleRate;
        for (Voice& v : voices_)
        {
            v.amp.setSampleRate(sampleRate); v.mod.setSampleRate(sampleRate);
            v.f1.setSampleRate(sampleRate);  v.f2.setSampleRate(sampleRate);
        }
        controllers_.prepare(sampleRate, 10.0f);
        chorus_.prepare(sampleRate);
        delay_.prepare(sampleRate);
        reverb_.prepare(sampleRate);
        limiter_.prepare(sampleRate);
        volume_.setTime(20.0f, sampleRate);
        volume_.snap(dbToGain(get(kVolume)));
    }

    void reset() override
    {
        for (int i = 0; i < kVoices; ++i)
        {
            Voice& v = voices_[i];
            v.active = v.gate = v.sustained = false; v.note = -1;
            v.amp.reset(); v.mod.reset(); v.fm.reset(); v.f1.reset(); v.f2.reset();
            v.rng.reseed(0x9E3779B9u * (uint32_t)(i + 1));
        }
        monoCount_ = 0; lastNote_ = -1;
        arpClear(); arpHeldCount_ = 0; arpKeysDown_ = 0;
        controllers_.reset();
        eq_.reset(); eqR_.reset(); chorus_.reset(); delay_.reset(); reverb_.reset(); limiter_.reset();
        distIn_ = distOut_ = 0.0f; distGain_ = 1.0f;
        panPhase_ = 0.0f; globalLfoPhase_[0] = globalLfoPhase_[1] = 0.0f;
        volume_.snap(dbToGain(get(kVolume)));
    }

    void process(mpc::ProcessContext& ctx) override
    {
        readSettings(ctx);

        int pos = 0;
        for (int e = 0; e < ctx.numMidiEvents; ++e)
        {
            const mpc::MidiEvent& ev = ctx.midi[e];
            if (ev.sampleOffset > pos) { render(ctx.outputs[0], ctx.outputs[1], pos, ev.sampleOffset); pos = ev.sampleOffset; }
            handleMidi(ev);
        }
        render(ctx.outputs[0], ctx.outputs[1], pos, ctx.numFrames);

        applyEffects(ctx);
    }

private:
    static constexpr int kVoices = 8;
    static constexpr int kCtrl = 16;      // control-rate interval (samples)
    static constexpr int kChunk = 64;

    struct Voice
    {
        bool active = false, gate = false, sustained = false;
        int note = -1;
        uint32_t age = 0;
        float vel = 1.0f, detune = 0.0f, pitch = 60.0f, pitchTarget = 60.0f;
        float drift = 0.0f, driftTarget = 0.0f;
        int driftCount = 0, ctr = 0;
        Osc osc[3];
        Env amp, mod;
        ADEnv fm;
        ResoSVF f1, f2;
        VoiceLfo lfo[2];
        float lfoVal[2] = {0.0f, 0.0f}, lfoFade[2] = {1.0f, 1.0f};
        Xorshift32 rng;
        // control-rate results
        float pw[3] = {0.0f, 0.0f, 0.0f}, lvl[3] = {0.0f, 0.0f, 0.0f};
        float fmIndex = 0.0f, ampVel = 1.0f, modVel = 1.0f, inGain = 1.0f, bpGain = 1.0f;
        float gL = 1.0f, gR = 1.0f;   // unison level compensation + stereo spread
    };

    // Per-block snapshot of everything the audio code needs.
    struct Settings
    {
        int mode = 2, unison = 1, wave[3] = {2, 2, 2}, oct[3] = {0, 0, 0}, ftype = 0, slope = 1;
        int l1wave = 1, l2wave = 1, arpMode = 0, arpOct = 1;
        bool sync = false, l1key = false, l2key = false, arpOn = false, arpLatch = false;
        float uniDetune = 0, drift = 0, portaCoef = 1.0f, bendRange = 2.0f;
        float semi[3] = {0, 0, 0}, lfoSemis[3] = {0, 0, 0}, envSemis[3] = {0, 0, 0};
        float pwPos[3] = {0, 0, 0}, pwLfo[3] = {0, 0, 0}, pwEnv[3] = {0, 0, 0};
        float lvl[3] = {1, 0, 0}, noise = 0, ring = 0, fmLvl = 0, fmEnv = 0;
        float cutoffOct = 9.9f, reso = 0, qnorm = 0, keyTrack = 0, fEnvOct = 0, fLfoOct = 0;
        float aVel = 0, mVel = 0;
        float lfoCycles[2] = {0, 0}, lfoFadeStep[2] = {1, 1};
        float mwVibSemis = 0, atVibSemis = 0, mwFiltOct = 0, mwFLfoOct = 0, atFiltOct = 0;
        float arpGate = 0.5f, bpm = 120.0f;
        int arpRate = 3;
        float mixGain = 1.0f, dist = 0.0f, pan = 0.0f, panDepth = 0.0f, panInc = 0.0f;
        float l1o1 = 0, l2o1 = 0, l2o2 = 0, ado3 = 0, fVelOct = 0;
        bool uni[2] = {false, false};
    } s_;

    // ------------------------------------------------------------------ settings
    void readSettings(mpc::ProcessContext& ctx)
    {
        Settings& s = s_;
        s.mode = getInt(kVoiceMode);
        s.unison = clamp(getInt(kUnison), 1, kVoices);
        s.uniDetune = get(kUniDetune) / 127.0f * 0.5f;      // semitones at the outer voices
        s.drift = get(kDrift) / 127.0f * 0.35f;
        const float porta = get(kPorta);
        if (porta <= 0.0f) s.portaCoef = 1.0f;
        else
        {
            const float ms = 2.0f * std::exp2(porta / 127.0f * 11.0f);
            s.portaCoef = 1.0f - std::exp(-(float)kCtrl / (ms * 0.001f * sr_));
        }
        s.bendRange = get(kBendRange);
        s.sync = getBool(kSync);
        for (int o = 0; o < 3; ++o)
        {
            const int b = kO1Wave + o * kOscStride;
            s.wave[o] = getInt(b + fWave);
            s.semi[o] = 12.0f * (float)getInt(b + fOct) + (float)getInt(b + fSemi) + get(b + fCent) * 0.01f;
            s.lfoSemis[o] = pitchDepthSemis(get(b + fLfo));
            s.envSemis[o] = pitchDepthSemis(get(b + fEnv));
            s.pwPos[o] = get(b + fPw) / 64.0f;
            s.pwLfo[o] = get(b + fPwLfo) / 64.0f;
            s.pwEnv[o] = get(b + fPwEnv) / 64.0f;
        }
        for (int o = 0; o < 3; ++o) s.lvl[o] = get(kO1Lvl + o) / 127.0f;
        s.noise = get(kNoiseLvl) / 127.0f;
        s.ring = get(kRingLvl) / 127.0f;
        s.fmLvl = get(kFmLvl) / 127.0f * 0.6f;
        s.fmEnv = get(kFmEnv) / 64.0f * 0.9f;

        s.ftype = getInt(kFType);
        s.slope = getInt(kFSlope);
        s.cutoffOct = cutoffOct(get(kCutoff));
        s.reso = get(kReso) / 127.0f;
        overdrive_.set(get(kDrive));
        s.qnorm = get(kQNorm) / 127.0f;
        s.keyTrack = get(kKeyTrack) / 127.0f;
        s.fEnvOct = get(kFEnv) / 64.0f * 8.0f;
        s.fLfoOct = get(kFLfo) / 64.0f * 5.0f;
        s.aVel = get(kAVel);
        s.mVel = get(kMVel);
        s.l1o1 = get(kMixL1O1) / 64.0f;
        s.l2o1 = get(kMixL2O1) / 64.0f;
        s.l2o2 = get(kMixL2O2) / 64.0f;
        s.ado3 = get(kMixAdO3) / 64.0f;
        s.fVelOct = get(kFiltVel) / 64.0f * 3.0f;
        s.uni[0] = getBool(kL1Uni);
        s.uni[1] = getBool(kL2Uni);

        // BPM: the MPC's tempo when it provides one, else the patch's arp tempo.
        s.bpm = (ctx.transport.tempoValid && ctx.transport.bpm > 20.0) ? (float)ctx.transport.bpm : get(kArpTempo);

        const int lw[2] = {getInt(kL1Wave), getInt(kL2Wave)};
        s.l1wave = lw[0]; s.l2wave = lw[1];
        s.l1key = getBool(kL1Key); s.l2key = getBool(kL2Key);
        for (int l = 0; l < 2; ++l)
        {
            const int base = l == 0 ? kL1Wave : kL2Wave;
            const int sync = getInt(base + 3);
            float hz = lfoHz(get(base + 1));
            if (sync > 0) hz = s.bpm / 60.0f / syncBeats(sync);
            s.lfoCycles[l] = hz / sr_ * (float)kCtrl;
            const float delay = get(base + 2);
            s.lfoFadeStep[l] = delay <= 0.0f ? 1.0f : (float)kCtrl / (envTimeMs(delay) * 0.001f * sr_ * 2.0f);
            globalLfoPhase_[l] = wrapPhase(globalLfoPhase_[l] + hz / sr_ * (float)ctx.numFrames);
        }

        s.mwVibSemis = 0.5f * pitchDepthSemis(get(kMwVib));   // wheel / aftertouch vibrato: finer scale
        s.atVibSemis = 0.5f * pitchDepthSemis(get(kAtVib));
        s.mwFiltOct = get(kMwFilt) / 64.0f * 5.0f;
        s.mwFLfoOct = get(kMwFLfo) / 64.0f * 5.0f;
        s.atFiltOct = get(kAtFilt) / 64.0f * 5.0f;

        const bool arpWasOn = s.arpOn;
        s.arpOn = getBool(kArpOn);
        s.arpMode = getInt(kArpMode);
        s.arpOct = clamp(getInt(kArpOct), 1, 4);
        s.arpRate = getInt(kArpRate);
        s.arpGate = get(kArpGate);
        s.arpLatch = getBool(kArpLatch);
        if (arpWasOn && !s.arpOn) { arpReleaseSounding(); arpHeldCount_ = 0; arpKeysDown_ = 0; }
        if (!s.arpLatch && arpKeysDown_ == 0 && arpHeldCount_ > 0) { arpHeldCount_ = 0; arpReleaseSounding(); }

        const float aa = envAttackMs(get(kAAtk)), ad = envTauMs(get(kADec)), as = get(kASus) / 127.0f, ar = envTauMs(get(kARel));
        const float ma = envAttackMs(get(kMAtk)), md = envTauMs(get(kMDec)), ms = get(kMSus) / 127.0f, mr = envTauMs(get(kMRel));
        const float fa = envTimeMs(get(kFmAtk)), fd = envTimeMs(get(kFmDec)) * 2.0f;
        for (Voice& v : voices_)
        {
            v.amp.set(aa, ad, as, ar);
            v.mod.set(ma, md, ms, mr);
            v.fm.set(fa, fd, sr_);
        }

        // Effects
        s.mixGain = dbToGain(get(kMixLevel) * 0.5f);
        s.dist = get(kDist) / 127.0f;
        eq_.set(get(kEqLevel), get(kEqFreq), sr_);
        eqR_.set(get(kEqLevel), get(kEqFreq), sr_);
        chorus_.set(getInt(kChType), get(kChSend), get(kChRate), get(kChDepth), get(kChFb));
        float dlMs = 5.0f + get(kDlTime) * 5.3f;
        const int dlSync = getInt(kDlSync);
        if (dlSync > 0) dlMs = syncBeats(dlSync) * 60000.0f / s.bpm;
        delay_.set(get(kDlSend), dlMs, get(kDlFb), get(kDlWidth));
        reverb_.set(getInt(kRvType), get(kRvSend), get(kRvDecay));
        s.pan = get(kPan) / 64.0f;
        s.panDepth = get(kPanDepth) / 127.0f;
        s.panInc = 0.05f * std::exp2(get(kPanRate) / 127.0f * 9.0f) / sr_;
        volume_.setTarget(dbToGain(get(kVolume)));
    }

    // ------------------------------------------------------------------ MIDI
    void handleMidi(const mpc::MidiEvent& ev)
    {
        if (ev.isNoteOn())        keyOn(ev.note(), ev.velocity());
        else if (ev.isNoteOff())  keyOff(ev.note());
        else if (ev.isAllNotesOff())
        {
            arpReleaseSounding(); arpHeldCount_ = 0; arpKeysDown_ = 0;
            monoCount_ = 0;
            for (Voice& v : voices_) releaseVoice(v);
        }
        else if (ev.isController() && ev.controller() == cc::Sustain)
        {
            controllers_.handle(ev);
            if (!controllers_.sustain())
                for (Voice& v : voices_) if (v.sustained) { v.sustained = false; releaseVoice(v); }
        }
        else controllers_.handle(ev);   // mod wheel, aftertouch, pitch bend
    }

    // Keyboard notes go to the arpeggiator when it is on, else straight to the engine.
    void keyOn(int note, int vel)
    {
        if (!s_.arpOn) { engineNoteOn(note, vel); return; }
        if (arpKeysDown_ == 0 && (s_.arpLatch || arpHeldCount_ == 0))
        {
            // First key of a new phrase: clear a latched chord and restart the pattern.
            arpHeldCount_ = 0;
            arpReleaseSounding();
            arpStep_ = 0;
            arpCounter_ = 0.0;   // fire the first step now
        }
        ++arpKeysDown_;
        for (int i = 0; i < arpHeldCount_; ++i) if (arpHeld_[i] == note) return;
        if (arpHeldCount_ < kMaxArpNotes) { arpHeld_[arpHeldCount_] = note; arpVel_[arpHeldCount_] = vel; ++arpHeldCount_; }
    }

    void keyOff(int note)
    {
        if (!s_.arpOn) { engineNoteOff(note); return; }
        if (arpKeysDown_ > 0) --arpKeysDown_;
        if (s_.arpLatch) return;
        for (int i = 0; i < arpHeldCount_; ++i)
            if (arpHeld_[i] == note)
            {
                for (int k = i; k + 1 < arpHeldCount_; ++k) { arpHeld_[k] = arpHeld_[k + 1]; arpVel_[k] = arpVel_[k + 1]; }
                --arpHeldCount_;
                break;
            }
        if (arpHeldCount_ == 0) arpReleaseSounding();
    }

    // ------------------------------------------------------------------ voice engine
    bool isMono() const { return s_.mode <= 1; }

    void engineNoteOn(int note, int vel)
    {
        const int U = s_.unison;
        if (isMono())
        {
            const bool legato = monoCount_ > 0;
            if (monoCount_ < kMaxMono) monoStack_[monoCount_++] = note;
            if (!legato)
            {
                const int from = (s_.mode == 0 && s_.portaCoef < 1.0f) ? lastNote_ : -1;
                for (int k = 0; k < U; ++k) startVoice(voices_[k], note, vel, k, U, from, true);
            }
            else monoLegato(note);
            lastNote_ = note;
            return;
        }

        int chosen = 0;
        if (s_.mode == 3)   // Poly 2: replay the same note on the same voices
            for (Voice& v : voices_)
                if (v.active && v.note == note && chosen < U)
                    startVoice(v, note, vel, chosen++, U, -1, true);
        bool used[kVoices] = {};
        for (int k = chosen; k < U; ++k)
        {
            const int vi = pickVoice(used);
            used[vi] = true;
            startVoice(voices_[vi], note, vel, k, U, s_.portaCoef < 1.0f ? lastNote_ : -1, true);
        }
        lastNote_ = note;
    }

    void engineNoteOff(int note)
    {
        if (isMono())
        {
            int idx = -1;
            for (int i = 0; i < monoCount_; ++i) if (monoStack_[i] == note) idx = i;
            if (idx < 0) return;
            const bool wasTop = idx == monoCount_ - 1;
            for (int i = idx; i + 1 < monoCount_; ++i) monoStack_[i] = monoStack_[i + 1];
            --monoCount_;
            if (monoCount_ == 0)
            {
                for (int k = 0; k < s_.unison; ++k)
                {
                    Voice& v = voices_[k];
                    if (controllers_.sustain()) v.sustained = true; else releaseVoice(v);
                }
            }
            else if (wasTop) monoLegato(monoStack_[monoCount_ - 1]);
            return;
        }
        for (Voice& v : voices_)
            if (v.active && v.gate && v.note == note)
            {
                if (controllers_.sustain()) v.sustained = true; else releaseVoice(v);
            }
    }

    // Mono: move the sounding voices to a new note without a new attack.
    void monoLegato(int note)
    {
        for (int k = 0; k < s_.unison; ++k)
        {
            Voice& v = voices_[k];
            if (!v.active) { startVoice(v, note, 100, k, s_.unison, -1, true); continue; }
            v.note = note;
            v.pitchTarget = (float)note;
            if (s_.mode == 1 && s_.portaCoef >= 1.0f) v.pitch = (float)note;   // Mono AG with no glide time
            if (s_.portaCoef >= 1.0f && s_.mode == 0) v.pitch = (float)note;
            v.gate = true; v.sustained = false;
            if (s_.mode == 0) { v.amp.noteOn(); v.mod.noteOn(); v.fm.trigger(); restartColour(v); }   // Mono re-triggers, Mono AG does not
        }
        lastNote_ = note;
    }

    int pickVoice(const bool* used) const
    {
        int best = -1;
        for (int i = 0; i < kVoices; ++i) if (!used[i] && !voices_[i].active) return i;
        for (int i = 0; i < kVoices; ++i)      // oldest releasing voice
            if (!used[i] && !voices_[i].gate && (best < 0 || voices_[i].age < voices_[best].age)) best = i;
        if (best >= 0) return best;
        for (int i = 0; i < kVoices; ++i)      // oldest held voice
            if (!used[i] && (best < 0 || voices_[i].age < voices_[best].age)) best = i;
        return best < 0 ? 0 : best;
    }

    void startVoice(Voice& v, int note, int vel, int k, int U, int glideFrom, bool trigger)
    {
        const bool wasActive = v.active;
        v.active = true; v.gate = true; v.sustained = false;
        v.note = note; v.age = ++ageCounter_;
        v.vel = (float)vel / 127.0f;
        v.detune = U > 1 ? ((float)k / (float)(U - 1) * 2.0f - 1.0f) * s_.uniDetune : 0.0f;
        v.pitchTarget = (float)note;
        if (!wasActive) v.pitch = glideFrom >= 0 ? (float)glideFrom : (float)note;
        else if (glideFrom < 0) v.pitch = (float)note;
        v.ampVel = velocityGain(s_.aVel, v.vel);
        v.modVel = velocityGain(s_.mVel, v.vel);
        {
            // Unison: keep the total level steady (1/sqrt(U)) and spread the voices across
            // the stereo field (outer voices at +-60 %), equal-power.
            const float spread = U > 1 ? ((float)k / (float)(U - 1) * 2.0f - 1.0f) * 0.6f : 0.0f;
            const float ang = (spread + 1.0f) * 0.25f * kPi;
            const float g = 1.0f / std::sqrt((float)U);
            v.gL = std::cos(ang) * kSqrt2 * g; v.gR = std::sin(ang) * kSqrt2 * g;
        }
        if (!wasActive)
        {
            // Unison voices start at random phases: in-phase copies would add up to a sudden
            // level jump (and clipping) instead of a wide, chorused sound.
            for (int o = 0; o < 3; ++o) { v.osc[o].phase = (o == 0 && U == 1) ? 0.0f : v.rng.nextFloat(); v.osc[o].n1 = v.osc[o].n2 = 0.0f; }
            v.f1.reset(); v.f2.reset();
            v.driftTarget = v.rng.nextBipolar() * s_.drift;
        }
        const bool keys[2] = {s_.l1key, s_.l2key};
        for (int l = 0; l < 2; ++l)
        {
            if (keys[l]) v.lfo[l].restart(0.0f);
            else if (!wasActive) v.lfo[l].restart(globalLfoPhase_[l]);
            v.lfoFade[l] = s_.lfoFadeStep[l] >= 1.0f ? 1.0f : 0.0f;
        }
        if (trigger) { v.amp.noteOn(); v.mod.noteOn(); v.fm.trigger(); restartColour(v); }
        v.ctr = 0;
    }

    // Table waves start bright (harder playing = brighter) and fade to their body.
    static void restartColour(Voice& v)
    {
        for (int o = 0; o < 3; ++o) v.osc[o].bright = 0.35f + 0.65f * v.vel;
    }

    void releaseVoice(Voice& v)
    {
        if (!v.active) return;
        v.gate = false; v.sustained = false;
        v.amp.noteOff(); v.mod.noteOff();
    }

    // ------------------------------------------------------------------ arpeggiator
    static constexpr int kMaxArpNotes = 32;
    static constexpr int kMaxMono = 32;

    void arpClear() { arpSoundingCount_ = 0; arpGateLeft_ = -1.0; arpCounter_ = 0.0; arpStep_ = 0; }

    void arpReleaseSounding()
    {
        for (int i = 0; i < arpSoundingCount_; ++i) engineNoteOff(arpSounding_[i]);
        arpSoundingCount_ = 0;
        arpGateLeft_ = -1.0;
    }

    double arpStepSamples() const
    {
        return (double)syncBeats(s_.arpRate + 1) * 60.0 / (double)s_.bpm * (double)sr_;
    }

    // Samples until the arpeggiator needs to act (step or gate end); large if idle.
    int arpSamplesToEvent() const
    {
        if (!s_.arpOn || arpHeldCount_ == 0) return 1 << 30;
        double n = arpCounter_;
        if (arpGateLeft_ >= 0.0 && arpGateLeft_ < n) n = arpGateLeft_;
        const int i = (int)std::ceil(n);
        return i < 0 ? 0 : i;
    }

    void arpAdvance(int n)
    {
        if (!s_.arpOn || arpHeldCount_ == 0) return;
        arpCounter_ -= n;
        if (arpGateLeft_ >= 0.0)
        {
            arpGateLeft_ -= n;
            if (arpGateLeft_ <= 0.0) arpReleaseSounding();
        }
        if (arpCounter_ <= 0.0) { arpFireStep(); arpCounter_ += arpStepSamples(); if (arpCounter_ < 1.0) arpCounter_ = 1.0; }
    }

    void arpFireStep()
    {
        arpReleaseSounding();
        const int n = arpHeldCount_;
        if (n <= 0) return;
        int sorted[kMaxArpNotes], sortedVel[kMaxArpNotes];
        for (int i = 0; i < n; ++i) { sorted[i] = arpHeld_[i]; sortedVel[i] = arpVel_[i]; }
        if (s_.arpMode != 4)   // all modes but "Played" use pitch order
            for (int i = 1; i < n; ++i)
            {
                const int key = sorted[i], kv = sortedVel[i]; int j = i - 1;
                while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; sortedVel[j + 1] = sortedVel[j]; --j; }
                sorted[j + 1] = key; sortedVel[j + 1] = kv;
            }
        const int len = n * s_.arpOct;
        const int st = arpStep_++;
        if (s_.arpMode == 6)   // Chord
        {
            const int oct = st % s_.arpOct;
            for (int i = 0; i < n && arpSoundingCount_ < kMaxArpNotes; ++i)
                arpPlay(sorted[i] + 12 * oct, sortedVel[i]);
        }
        else
        {
            int idx = 0;
            switch (s_.arpMode)
            {
                case 0: idx = st % len; break;
                case 1: idx = len - 1 - st % len; break;
                case 2: { const int per = len > 1 ? 2 * len - 2 : 1; const int p = st % per; idx = p < len ? p : per - p; break; }
                case 3: { const int per = 2 * len; const int p = st % per; idx = p < len ? p : per - 1 - p; break; }
                case 4: idx = st % len; break;
                case 5: idx = (int)(arpRng_.nextFloat() * (float)len); if (idx >= len) idx = len - 1; break;
                default: idx = st % len; break;
            }
            const int note = sorted[idx % n] + 12 * (idx / n);
            arpPlay(note, sortedVel[idx % n]);
        }
        const float gate = s_.arpGate;
        arpGateLeft_ = gate >= 100.0f ? -1.0 : arpStepSamples() * (0.05 + 0.9 * (double)gate / 100.0);
    }

    void arpPlay(int note, int vel)
    {
        if (note > 127) note -= 12 * ((note - 116) / 12 + 1);
        engineNoteOn(note, vel);
        arpSounding_[arpSoundingCount_++] = note;
    }

    // ------------------------------------------------------------------ rendering
    void render(float* outL, float* outR, int from, int to)
    {
        while (from < to)
        {
            int n = to - from;
            if (n > kChunk) n = kChunk;
            const int toArp = arpSamplesToEvent();
            if (toArp == 0) { arpAdvance(0); continue; }
            if (toArp < n) n = toArp;

            for (int i = 0; i < n; ++i) controllers_.tick();
            for (int vi = 0; vi < kVoices; ++vi)
            {
                Voice& v = voices_[vi];
                if (!v.active) continue;
                renderVoice(v, outL + from, outR + from, n);
            }
            arpAdvance(n);
            from += n;
        }
    }

    void controlUpdate(Voice& v)
    {
        const Settings& s = s_;
        // glide and drift
        v.pitch += (v.pitchTarget - v.pitch) * s.portaCoef;
        if (--v.driftCount <= 0) { v.driftCount = (int)(sr_ * 0.4f / kCtrl); v.driftTarget = v.rng.nextBipolar() * s.drift; }
        v.drift += (v.driftTarget - v.drift) * 0.01f;

        // LFOs (per voice, with delayed onset)
        const int waves[2] = {s.l1wave, s.l2wave};
        for (int l = 0; l < 2; ++l)
        {
            float val = v.lfo[l].tick(waves[l], s.lfoCycles[l], v.rng);
            if (s.uni[l] && waves[l] != kLSteps) val = 0.5f * (val + 1.0f);
            if (v.lfoFade[l] < 1.0f) { v.lfoFade[l] += s.lfoFadeStep[l]; if (v.lfoFade[l] > 1.0f) v.lfoFade[l] = 1.0f; }
            v.lfoVal[l] = val * v.lfoFade[l];
        }
        const float l1 = v.lfoVal[0], l2 = v.lfoVal[1];
        const float menv = v.mod.level() * v.modVel;
        const float fmEnv = v.fm.tick(kCtrl);
        const float wheel = controllers_.modWheel(), at = controllers_.aftertouch();

        const float base = v.pitch + v.detune + v.drift + controllers_.pitchBend() * s.bendRange;
        const float vib = l1 * (s.mwVibSemis * wheel + s.atVibSemis * at);
        for (int o = 0; o < 3; ++o)
        {
            const float semis = base + s.semi[o] + l1 * s.lfoSemis[o] + menv * s.envSemis[o] + vib;
            float inc = 440.0f * std::exp2((semis - 69.0f) * (1.0f / 12.0f)) / sr_;
            v.osc[o].inc = inc > 0.45f ? 0.45f : inc;
            v.osc[o].tableLevel = WaveTables::levelFor(v.osc[o].inc);
            if (s.wave[o] >= kFirstTable && v.osc[o].bright > 0.001f)
            {
                const float secs = WaveTables::fadeSeconds(s.wave[o] - kFirstTable) * std::exp2((60.0f - v.pitch) / 24.0f);
                v.osc[o].bright *= std::exp(-(float)kCtrl / (secs * sr_));
            }
            v.pw[o] = clamp(s.pwPos[o] + l2 * s.pwLfo[o] + menv * s.pwEnv[o], -1.0f, 1.0f);
        }
        v.fmIndex = s.fmLvl + s.fmEnv * fmEnv;
        v.lvl[0] = clamp(s.lvl[0] + s.l1o1 * l1 + s.l2o1 * l2, 0.0f, 1.0f);
        v.lvl[1] = clamp(s.lvl[1] + s.l2o2 * l2, 0.0f, 1.0f);
        v.lvl[2] = clamp(s.lvl[2] + s.ado3 * fmEnv, 0.0f, 1.0f);

        float oct = s.cutoffOct + menv * s.fEnvOct + l2 * s.fLfoOct + s.keyTrack * (v.pitch - 60.0f) / 12.0f
                  + wheel * (s.mwFiltOct + l2 * s.mwFLfoOct) + at * s.atFiltOct + s.fVelOct * v.vel;
        oct = clamp(oct, 0.0f, 10.2f);
        const float hz = 20.0f * std::exp2(oct);
        // Damping: 12 dB = one stage from Butterworth (1.414) down past zero (self-oscillation);
        // 24 dB = Butterworth pair, the second stage carrying the resonance.
        const float rc = std::pow(s.reso, 0.75f);
        float k1, k2 = 2.0f;
        if (s.slope == 1) { k1 = 1.848f; k2 = 0.765f - (0.765f + 0.12f) * rc; }
        else              { k1 = 1.414f - (1.414f + 0.12f) * rc; }
        v.f1.set(hz, k1);
        if (s.slope == 1) v.f2.set(hz, k2);
        // Band pass: scale by 1/Q so the peak sits at unity gain, like the other modes.
        v.bpGain = s.ftype == 1 ? std::fmax(k1, 0.08f) * (s.slope == 1 ? std::fmax(k2, 0.08f) : 1.0f) : 1.0f;
        // Q normalise: pull the level down as resonance rises (classic ladder feel).
        v.inGain = 1.0f - 0.6f * s.qnorm * s.reso;
    }

    void renderVoice(Voice& v, float* outL, float* outR, int n)
    {
        const Settings& s = s_;
        const WaveTables& tables = WaveTables::instance();
        const bool fmOn = s.fmLvl > 0.0f || s.fmEnv != 0.0f;
        const bool need0 = s.lvl[0] > 0.0f || s.l1o1 != 0.0f || s.l2o1 != 0.0f || s.ring > 0.0f || s.sync;
        const bool need1 = s.lvl[1] > 0.0f || s.l2o2 != 0.0f || s.ring > 0.0f || fmOn;
        const bool need2 = s.lvl[2] > 0.0f || s.ado3 != 0.0f;
        const bool noise = s.noise > 0.0f;
        const int mode = clamp(s.ftype, 0, 2);
        const bool two = s.slope == 1;
        constexpr float kVoiceGain = 0.28f;

        for (int i = 0; i < n; ++i)
        {
            if ((v.ctr++ % kCtrl) == 0) controlUpdate(v);
            v.mod.process();

            Osc& o0 = v.osc[0]; Osc& o1 = v.osc[1]; Osc& o2 = v.osc[2];
            o0.advance(); o1.advance(); o2.advance();
            if (s.sync && o0.wrapped && o0.inc > 0.0f)
                o1.phase = wrapPhase(o0.phase * (o1.inc / o0.inc));

            float a = 0.0f, b = 0.0f, c = 0.0f;
            if (need0) a = o0.render(s.wave[0], v.pw[0], 0.0f, v.rng, tables);
            if (need1) b = o1.render(s.wave[1], v.pw[1], 0.0f, v.rng, tables);
            if (need2) c = o2.render(s.wave[2], v.pw[2], fmOn ? v.fmIndex * b : 0.0f, v.rng, tables);

            float mix = a * v.lvl[0] + b * v.lvl[1] + c * v.lvl[2] + a * b * s.ring;
            if (noise) mix += v.rng.nextBipolar() * s.noise;
            mix = overdrive_.process(mix * v.inGain);

            float y = v.f1.process(mix, mode);
            if (two) y = v.f2.process(y, mode);
            y *= v.bpGain;

            const float out = y * v.amp.process() * v.ampVel * kVoiceGain;
            outL[i] += out * v.gL;
            outR[i] += out * v.gR;
        }
        if (!v.amp.isActive()) { v.active = false; v.gate = false; v.sustained = false; }
    }

    // ------------------------------------------------------------------ effects
    void applyEffects(mpc::ProcessContext& ctx)
    {
        float* L = ctx.outputs[0];
        float* R = ctx.numOutputs > 1 ? ctx.outputs[1] : nullptr;
        const Settings& s = s_;
        // Distortion: progressive drive (gentle at first, heavy at the top) with automatic
        // level matching, so the knob changes the grit, not the volume.
        const float drive = 1.0f + 40.0f * s.dist * s.dist;
        const float envA = 1.0f - std::exp(-1.0f / (0.06f * sr_));    // 60 ms level followers
        float gl = 1.0f, gr = 1.0f;
        for (int i = 0; i < ctx.numFrames; ++i)
        {
            if ((i & 31) == 0)
            {
                panPhase_ += s.panInc * 32.0f; if (panPhase_ >= 1.0f) panPhase_ -= 1.0f;
                const float p = clamp(s.pan + s.panDepth * WavetableSine::lookup(panPhase_), -1.0f, 1.0f);
                const float ang = (p + 1.0f) * 0.25f * kPi;
                gl = std::cos(ang) * kSqrt2; gr = std::sin(ang) * kSqrt2;
            }
            float l = L[i] * s.mixGain, r = (R ? R[i] : L[i]) * s.mixGain;
            if (s.dist > 0.0f)
            {
                const float dl = fastTanh(l * drive), dr = fastTanh(r * drive);
                distIn_ += envA * (0.5f * (l * l + r * r) - distIn_);
                distOut_ += envA * (0.5f * (dl * dl + dr * dr) - distOut_);
                const float target = distOut_ > 1e-9f ? clamp(std::sqrt(distIn_ / distOut_), 0.05f, 1.5f) : 1.0f;
                distGain_ += envA * (target - distGain_);
                l = dl * distGain_; r = dr * distGain_;
            }
            l = eq_.process(l) * gl;
            r = eqR_.process(r) * gr;
            if (chorus_.active()) chorus_.process(l, r);
            delay_.process(l, r);
            reverb_.process(l, r);
            const float g = volume_.process();
            l = sanitize(l * g); r = sanitize(r * g);
            limiter_.process(l, r);
            L[i] = flushDenormal(l);
            if (R) R[i] = flushDenormal(r);
        }
    }

    Voice voices_[kVoices];
    uint32_t ageCounter_ = 0;
    int monoStack_[kMaxMono] = {}, monoCount_ = 0, lastNote_ = -1;

    int arpHeld_[kMaxArpNotes] = {}, arpVel_[kMaxArpNotes] = {}, arpHeldCount_ = 0, arpKeysDown_ = 0;
    int arpSounding_[kMaxArpNotes] = {}, arpSoundingCount_ = 0, arpStep_ = 0;
    double arpCounter_ = 0.0, arpGateLeft_ = -1.0;
    Xorshift32 arpRng_{0x2545F491u};

    MidiControllers controllers_;
    TiltEQ eq_, eqR_;
    ChorusPhaser chorus_;
    StereoDelay delay_;
    Reverb reverb_;
    PeakLimiter limiter_;
    Overdrive overdrive_;
    Smoother volume_;
    float sr_ = 44100.0f, panPhase_ = 0.0f, globalLfoPhase_[2] = {0.0f, 0.0f};
    float distIn_ = 0.0f, distOut_ = 0.0f, distGain_ = 1.0f;   // distortion level matching
};

MPC_PLUGIN_ENTRY(Jamiro)
