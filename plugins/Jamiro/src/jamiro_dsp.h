// jamiro_dsp.h - building blocks for the Jamiro synth: value curves, oscillator
// wavetables, per-voice LFO, AD envelope and the global effects (distortion, EQ,
// chorus/phaser, delay, reverb, auto-pan).
//
// SPDX-License-Identifier: MIT
//
// All code here is original. The reverb is a classic Schroeder/Moorer network
// (parallel damped combs + series allpasses), a textbook public-domain design.
// No allocation happens outside prepare().

#ifndef JAMIRO_DSP_H
#define JAMIRO_DSP_H

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "dsp/math.h"
#include "dsp/noise.h"
#include "dsp/oscillator.h"
#include "dsp/delay_line.h"

namespace jamiro {

using mpc::dsp::clamp;
using mpc::dsp::kPi;
using mpc::dsp::kTwoPi;
using mpc::dsp::wrapPhase;
using mpc::dsp::fastTanh;

// ---------------------------------------------------------------------------
// Value curves. Every sound parameter uses the same 0..127 (or -64..+63)
// numbers as the hardware it is inspired by; these turn them into units.
// ---------------------------------------------------------------------------

// Envelope / glide segment time: 0 -> 0.5 ms, 127 -> 20 s (exponential).
inline float envTimeMs(float v) { return 0.5f * std::pow(40000.0f, clamp(v, 0.0f, 127.0f) / 127.0f); }

// Filter cutoff in octaves above 20 Hz: 0 -> 20 Hz, 127 -> ~19 kHz.
inline float cutoffOct(float v) { return v * (9.9f / 127.0f); }

// LFO speed: 0 -> 0.05 Hz, 68 -> ~5 Hz, 127 -> ~270 Hz (capped at 120 Hz).
inline float lfoHz(float v) { float hz = 0.05f * std::exp2(clamp(v, 0.0f, 127.0f) * 0.0976f); return hz > 120.0f ? 120.0f : hz; }

// Bipolar pitch depth (-64..63) -> semitones. 30 = exactly one octave at full LFO
// swing (that is what the "chromatic" LFO trick relies on), small values give a
// gentle vibrato.
inline float pitchDepthSemis(float d)
{
    const float a = std::fabs(d) / 30.0f;
    float s = 12.0f * a * a;
    if (s > 48.0f) s = 48.0f;
    return d < 0.0f ? -s : s;
}

// Velocity response: depth -64..63 -> gain for a velocity of 0..1.
inline float velocityGain(float depth, float vel01)
{
    const float d = clamp(depth / 64.0f, -1.0f, 1.0f);
    float g = d >= 0.0f ? 1.0f - d * (1.0f - vel01) : 1.0f + d * vel01;
    return g < 0.0f ? 0.0f : g;
}

// Tempo-synced lengths in quarter notes. Index 0 of the LFO/delay tables is "Off".
inline float syncBeats(int index)
{
    // 0 Off, then 32T 32 16T 16 8T 16D 8 4T 8D 4 2T 4D 2 1barT 2D 1bar 2barT 1barD 2bars
    // 4barT 3bars 5barT 4bars 3barD 7barT 5bars 8barT 6bars 7bars 5barD 8bars 9bars 7barD 12bars
    static const float kBeats[] = {
        0.0f, 1.0f / 12.0f, 0.125f, 1.0f / 6.0f, 0.25f, 1.0f / 3.0f, 0.375f, 0.5f, 2.0f / 3.0f, 0.75f, 1.0f,
        4.0f / 3.0f, 1.5f, 2.0f, 8.0f / 3.0f, 3.0f, 4.0f, 16.0f / 3.0f, 6.0f, 8.0f,
        32.0f / 3.0f, 12.0f, 40.0f / 3.0f, 16.0f, 18.0f, 56.0f / 3.0f, 20.0f, 64.0f / 3.0f, 24.0f, 28.0f,
        30.0f, 32.0f, 36.0f, 42.0f, 48.0f };
    const int n = (int)(sizeof(kBeats) / sizeof(kBeats[0]));
    return kBeats[clamp(index, 0, n - 1)];
}

inline std::vector<std::string> lfoSyncNames()
{
    return { "Off", "32nd T", "32nd", "16th T", "16th", "8th T", "16th D", "8th", "4th T", "8th D", "4th",
             "2nd T", "4th D", "2nd", "1 bar T", "2nd D", "1 bar", "2 bars T", "1 bar D", "2 bars",
             "4 bars T", "3 bars", "5 bars T", "4 bars", "3 bars D", "7 bars T", "5 bars", "8 bars T", "6 bars",
             "7 bars", "5 bars D", "8 bars", "9 bars", "7 bars D", "12 bars" };
}
inline std::vector<std::string> delaySyncNames()
{
    std::vector<std::string> v = lfoSyncNames();
    v.resize(20);
    return v;
}
// Arpeggiator rates have no "Off": index i plays at syncBeats(i + 1).
inline std::vector<std::string> arpRateNames()
{
    std::vector<std::string> v = lfoSyncNames();
    return std::vector<std::string>(v.begin() + 1, v.begin() + 17);
}

// ---------------------------------------------------------------------------
// Sampled-style waveforms (Organ, E.Piano, Rhodes ...). We build our own tables by
// additive synthesis (no samples are used). Each instrument has TWO spectra, like a
// real recording: a bright ATTACK (hammer, tine ping, pluck, organ percussion) and a
// rounder BODY. Every note starts on the attack and fades to the body at an
// instrument-specific speed (faster for high notes, brighter for hard velocities).
// One band-limited version per octave keeps every harmonic below ~0.45 x sample
// rate, so the tone stays even up the keyboard without aliasing. Both spectra of a
// table are scaled to the same loudness, so the fade changes colour, not level.
// ---------------------------------------------------------------------------
class WaveTables
{
public:
    static constexpr int kSize = 2048;
    static constexpr int kTables = 9;
    static constexpr int kLevels = 10;       // one per octave, from 27.5 Hz (A0) up
    static constexpr int kMaxHarmonics = 160;

    // Build once (call from the plugin constructor, never from the audio thread).
    static const WaveTables& instance() { static WaveTables t; return t; }

    // Band-limit level for a phase increment (cycles per sample). Cheap: call at control rate.
    static int levelFor(float inc)
    {
        float top = kBaseInc;
        for (int lv = 0; lv < kLevels - 1; ++lv, top *= 2.0f)
            if (inc <= top) return lv;
        return kLevels - 1;
    }

    // Seconds for the attack colour to fade (at middle C) for each table.
    static float fadeSeconds(int table)
    {
        static const float kFade[kTables] = { 0.28f, 1.1f, 0.55f, 0.35f, 0.7f, 1.3f, 0.8f, 0.6f, 0.4f };
        return kFade[clamp(table, 0, kTables - 1)];
    }

    // phase 0..1, table 0..8, level from levelFor(), bright 1 = attack .. 0 = body
    float read(int table, float phase, int level, float bright) const
    {
        const int tb = clamp(table, 0, kTables - 1), lv = clamp(level, 0, kLevels - 1);
        const float pos = phase * (float)kSize;
        int i = (int)pos;
        if (i >= kSize) i = kSize - 1;
        const float f = pos - (float)i;
        const float* b = data_[tb][0][lv];
        const float body = b[i] + f * (b[i + 1] - b[i]);
        if (bright <= 0.001f) return body;
        const float* a = data_[tb][1][lv];
        const float att = a[i] + f * (a[i + 1] - a[i]);
        return body + bright * (att - body);
    }

private:
    static constexpr float kBaseInc = 27.5f / 44100.0f;

    WaveTables()
    {
        // Sine lookup for fast additive synthesis (index = harmonic * sample, mod size).
        std::vector<double> sine((size_t)kSize);
        for (int i = 0; i < kSize; ++i) sine[(size_t)i] = std::sin((double)kTwoPi * (double)i / (double)kSize);
        std::vector<double> acc((size_t)kSize + 1);
        for (int tb = 0; tb < kTables; ++tb)
            for (int part = 0; part < 2; ++part)
            {
                float gain = 1.0f;
                for (int lv = 0; lv < kLevels; ++lv)
                {
                    const double topInc = (double)kBaseInc * std::pow(2.0, lv);
                    int maxH = (int)(0.45 / topInc);
                    if (maxH > kMaxHarmonics) maxH = kMaxHarmonics;
                    if (maxH < 1) maxH = 1;
                    std::fill(acc.begin(), acc.end(), 0.0);
                    for (int n = 1; n <= maxH; ++n)
                    {
                        const double a = amp(tb, part == 1, n);
                        if (a == 0.0) continue;
                        // Fixed, spread phases (same for attack and body so the fade is smooth).
                        const int ph = n == 1 ? 0 : (int)(std::fmod(0.6180339887 * n * n, 1.0) * kSize);
                        for (int i = 0; i < kSize; ++i)
                            acc[(size_t)i] += a * sine[(size_t)((n * i + ph) & (kSize - 1))];
                    }
                    acc[(size_t)kSize] = acc[0];
                    if (lv == 0)
                    {
                        double rms = 0.0, peak = 1e-9;
                        for (int i = 0; i < kSize; ++i) { rms += acc[(size_t)i] * acc[(size_t)i]; peak = std::fmax(peak, std::fabs(acc[(size_t)i])); }
                        rms = std::sqrt(rms / kSize);
                        gain = (float)std::fmin(0.58 / std::fmax(rms, 1e-9), 0.98 / peak);
                    }
                    float* t = data_[tb][part][lv];
                    for (int i = 0; i <= kSize; ++i) t[i] = clamp((float)acc[(size_t)i] * gain, -1.0f, 1.0f);
                }
            }
    }

    // Harmonic recipes (original, by ear). attack = true for the bright onset spectrum.
    static double amp(int table, bool attack, int n)
    {
        const double fn = (double)n;
        const double pi = (double)kPi;
        auto pick = [n](std::initializer_list<std::pair<int, double>> l) {
            for (const auto& p : l) if (p.first == n) return p.second;
            return 0.0;
        };
        switch (table)
        {
            case 0: // Organ: drawbars 8, 4, 2 2/3, 2, 1 3/5, 1 1/3, 1; attack adds the 3rd-harmonic percussion
            {
                const double body = pick({{1, 1.0}, {2, 0.8}, {3, 0.6}, {4, 0.55}, {5, 0.15}, {6, 0.35}, {8, 0.3},
                                          {10, 0.1}, {12, 0.12}, {16, 0.08}});
                return attack ? body + pick({{3, 1.2}, {6, 0.25}, {9, 0.15}}) : body;
            }
            case 1: // Harpsichord: very bright pluck settling to a softer string
                if (n > 120) return 0.0;
                return attack ? (0.25 + 0.75 * std::fabs(std::sin(pi * fn * 0.11))) / std::pow(fn, 0.55)
                              : (0.3 + 0.7 * std::fabs(std::sin(pi * fn * 0.11))) / std::pow(fn, 1.15);
            case 2: // E.Piano (FM-style tine piano): metallic ping at 7-8 and 13-15, then a sweet two-partial body
                return attack ? pick({{1, 1.0}, {2, 0.7}, {3, 0.3}, {4, 0.25}, {5, 0.15}, {6, 0.12}, {7, 0.45}, {8, 0.35},
                                      {9, 0.2}, {13, 0.25}, {14, 0.4}, {15, 0.25}, {21, 0.08}})
                              : pick({{1, 1.0}, {2, 0.45}, {3, 0.12}, {4, 0.06}, {5, 0.03}});
            case 3: // Slap bass: snappy string pop, then a round low body
                if (n > 80) return 0.0;
                return attack ? ((n >= 3 && n <= 12) ? 1.8 : 1.0) / std::pow(fn, 0.6)
                              : 1.0 / std::pow(fn, 1.5);
            case 4: // Rhodes: bark and bell tine on the attack, a warm almost-sine body
                return attack ? pick({{1, 1.0}, {2, 0.9}, {3, 0.55}, {4, 0.45}, {5, 0.3}, {6, 0.22}, {7, 0.35}, {8, 0.3},
                                      {9, 0.2}, {10, 0.12}, {13, 0.1}, {14, 0.15}})
                              : pick({{1, 1.0}, {2, 0.28}, {3, 0.09}, {4, 0.05}, {5, 0.02}});
            case 5: // Rhodes tine: strong bell partials ringing longer
                return attack ? pick({{1, 1.0}, {2, 0.4}, {3, 0.15}, {4, 0.35}, {6, 0.15}, {7, 0.6}, {9, 0.2},
                                      {11, 0.15}, {14, 0.4}, {17, 0.12}, {21, 0.1}})
                              : pick({{1, 1.0}, {2, 0.25}, {4, 0.1}, {7, 0.12}});
            case 6: // Wurly: reedy bite with strong odd harmonics, mellowing out
                return attack ? pick({{1, 1.0}, {2, 0.5}, {3, 0.85}, {4, 0.35}, {5, 0.65}, {6, 0.25}, {7, 0.5},
                                      {8, 0.15}, {9, 0.35}, {10, 0.1}, {11, 0.25}, {13, 0.15}, {15, 0.1}})
                              : pick({{1, 1.0}, {2, 0.3}, {3, 0.4}, {4, 0.1}, {5, 0.15}, {7, 0.05}});
            case 7: // Clavinet: thin, nasal pickup comb, softening as the string decays
                if (n > 120) return 0.0;
                return std::pow(std::fabs(std::sin(pi * fn * 0.09)), 0.8) / std::pow(fn, attack ? 0.6 : 1.05);
            case 8: // Analog bass: resonant saw pluck closing to a soft round tone
                if (n > 96) return 0.0;
                return attack ? ((n >= 3 && n <= 6) ? 1.6 : 1.0) / (fn * (1.0 + (fn / 24.0) * (fn / 24.0)))
                              : 1.0 / (fn * (1.0 + (fn / 6.0) * (fn / 6.0)));
        }
        return 0.0;
    }

    float data_[kTables][2][kLevels][kSize + 1];
};

inline float polyBlep(float t, float dt)
{
    if (t < dt) { t /= dt; return t + t - t * t - 1.0f; }
    if (t > 1.0f - dt) { t = (t - 1.0f) / dt; return t * t + t + t + 1.0f; }
    return 0.0f;
}

// Oscillator waveforms (index == "Osc Wave" choice).
enum OscWave { kSine = 0, kTriangle, kSaw, kPulse, kWhite, kHPNoise, kBPNoise, kHBPNoise, kFirstTable };

inline std::vector<std::string> oscWaveNames()
{
    return { "Sine", "Triangle", "Saw", "Pulse", "White Noise", "HP Noise", "BP Noise", "HBP Noise",
             "Organ", "Harpsichord", "E.Piano", "Slap Bass", "Rhodes", "Rhodes Tine", "Wurly EP", "Clavinet", "Ana Bass" };
}

// One oscillator: phase accumulator + waveform evaluation. Noise waves keep a tiny filter state.
struct Osc
{
    float phase = 0.0f, inc = 0.0f;
    float n1 = 0.0f, n2 = 0.0f;   // noise filter states
    int tableLevel = 0;           // band-limit level for the table waves (set with inc)
    float bright = 0.0f;          // table waves: 1 = attack colour .. 0 = body (fades per note)
    bool wrapped = false;

    // Advance one sample. Returns true if the phase wrapped (for hard sync).
    void advance() { phase += inc; wrapped = false; if (phase >= 1.0f) { phase -= 1.0f; wrapped = true; } }

    // pw: -1..1 (0 = square / single saw). pm: phase offset in cycles (FM).
    float render(int wave, float pw, float pm, mpc::dsp::Xorshift32& rng, const WaveTables& tables)
    {
        const float dt = inc < 1e-6f ? 1e-6f : (inc > 0.45f ? 0.45f : inc);
        const float t = pm == 0.0f ? phase : wrapPhase(phase + pm);
        switch (wave)
        {
            case kSine: return mpc::dsp::WavetableSine::lookup(t);
            case kTriangle: return 1.0f - 4.0f * std::fabs(t - 0.5f);
            case kSaw:
            {
                float y = 2.0f * t - 1.0f - polyBlep(t, dt);
                const float a = std::fabs(pw);
                if (a > 0.001f)   // "double saw": a second saw half a cycle later, up to one octave
                {
                    float t2 = t + 0.5f; if (t2 >= 1.0f) t2 -= 1.0f;
                    y = (y + a * (2.0f * t2 - 1.0f - polyBlep(t2, dt))) / (1.0f + 0.5f * a);
                }
                return y;
            }
            case kPulse:
            {
                const float duty = 0.5f - 0.47f * std::fabs(pw);
                float y = t < duty ? 1.0f : -1.0f;
                y += polyBlep(t, dt);
                float t2 = t + 1.0f - duty; if (t2 >= 1.0f) t2 -= 1.0f;
                y -= polyBlep(t2, dt);
                return y;
            }
            case kWhite: return rng.nextBipolar();
            case kHPNoise: { const float x = rng.nextBipolar(); n1 += 0.15f * (x - n1); return (x - n1) * 1.2f; }
            case kBPNoise: { const float x = rng.nextBipolar(); n1 += 0.25f * (x - n1); n2 += 0.05f * (n1 - n2); return (n1 - n2) * 3.0f; }
            case kHBPNoise:{ const float x = rng.nextBipolar(); n1 += 0.6f * (x - n1); n2 += 0.25f * (n1 - n2); return (n1 - n2) * 2.0f; }
            default: return tables.read(wave - kFirstTable, t, tableLevel, bright);
        }
    }
};

// ---------------------------------------------------------------------------
// LFO waveforms (index == "LFO Wave" choice). Evaluated at control rate.
// ---------------------------------------------------------------------------
enum LfoWave { kLSine = 0, kLTri, kLSaw, kLSquare, kLSH, kLQuantSH, kLSmooth, kLExpDec, kLExpAtt, kLSteps, kNumLfoWaves };

inline std::vector<std::string> lfoWaveNames()
{
    return { "Sine", "Triangle", "Saw", "Square", "Random S/H", "Quant S/H", "Crossfade", "Exp Decay", "Exp Attack", "Scale Steps" };
}

struct VoiceLfo
{
    float phase = 0.0f, held = 0.0f, prevHeld = 0.0f, value = 0.0f;

    // Advance by `cycles` and return the new value (-1..1, Scale Steps is 0..1).
    float tick(int wave, float cycles, mpc::dsp::Xorshift32& rng)
    {
        phase += cycles;
        if (phase >= 1.0f)
        {
            phase -= std::floor(phase);
            prevHeld = held;
            held = rng.nextBipolar();
        }
        const float t = phase;
        switch (wave)
        {
            case kLSine:    value = mpc::dsp::WavetableSine::lookup(t); break;
            case kLTri:     value = t < 0.25f ? 4.0f * t : (t < 0.75f ? 2.0f - 4.0f * t : 4.0f * t - 4.0f); break;   // starts at 0, rising
            case kLSaw:     value = 2.0f * t - 1.0f; break;
            case kLSquare:  value = t < 0.5f ? 1.0f : -1.0f; break;
            case kLSH:      value = held; break;
            case kLQuantSH: value = std::floor(held * 4.0f + 0.5f) * 0.25f; break;
            case kLSmooth:  value = prevHeld + (held - prevHeld) * t; break;
            case kLExpDec:  value = 2.0f * std::exp(-5.0f * t) - 1.0f; break;
            case kLExpAtt:  value = 1.0f - 2.0f * std::exp(-5.0f * t); break;
            case kLSteps:   value = std::floor(t * 12.0f) * (1.0f / 12.0f); break;   // rising chromatic scale
            default:        value = 0.0f; break;
        }
        return value;
    }
    void restart(float ph) { phase = ph; }
};

// Attack / decay envelope (drives the FM amount).
struct ADEnv
{
    float level = 0.0f, attInc = 1.0f, decCoef = 0.999f;
    bool attacking = false;
    void set(float attackMs, float decayMs, float sr)
    {
        attInc = 1.0f / std::fmax(1.0f, attackMs * 0.001f * sr);
        decCoef = std::exp(-1.0f / std::fmax(1.0f, decayMs * 0.001f * sr * 0.25f));
    }
    void trigger() { attacking = true; }
    void reset() { level = 0.0f; attacking = false; }
    // Advance n samples at once (control rate).
    float tick(int n)
    {
        if (attacking)
        {
            level += attInc * (float)n;
            if (level >= 1.0f) { level = 1.0f; attacking = false; }
        }
        else level *= std::pow(decCoef, (float)n);
        return level;
    }
};

// ---------------------------------------------------------------------------
// Effects
// ---------------------------------------------------------------------------

// Tilt EQ: -64 = low-pass, 0 = flat, +63 = high-pass, around a corner frequency.
struct TiltEQ
{
    float a = 0.1f, z = 0.0f, amt = 0.0f;
    void set(float level, float freq, float sr)
    {
        const float hz = 100.0f * std::exp2(clamp(freq, 0.0f, 127.0f) / 127.0f * 7.0f);
        a = 1.0f - std::exp(-kTwoPi * hz / sr);
        amt = clamp(level / 64.0f, -1.0f, 1.0f);
    }
    float process(float x)
    {
        z += a * (x - z);
        if (amt < 0.0f) return x + (z - x) * (-amt);
        if (amt > 0.0f) return x + ((x - z) - x) * amt;
        return x;
    }
    void reset() { z = 0.0f; }
};

// Stereo chorus / phaser.
class ChorusPhaser
{
public:
    void prepare(double sr)
    {
        sr_ = (float)sr;
        dl_[0].prepare((int)(0.05 * sr) + 4);
        dl_[1].prepare((int)(0.05 * sr) + 4);
        reset();
    }
    void reset()
    {
        dl_[0].reset(); dl_[1].reset();
        for (int c = 0; c < 2; ++c) { fb_[c] = 0.0f; for (int s = 0; s < 4; ++s) ap_[c][s] = 0.0f; }
        phase_ = 0.0f;
    }
    // type 0 = chorus, 1 = phaser. mix/rate/depth 0..127, feedback -64..63
    void set(int type, float mix, float rate, float depth, float feedback)
    {
        type_ = type;
        mix_ = clamp(mix / 127.0f, 0.0f, 1.0f);
        inc_ = 0.05f * std::exp2(clamp(rate, 0.0f, 127.0f) / 127.0f * 8.0f) / sr_;
        depth_ = clamp(depth / 127.0f, 0.0f, 1.0f);
        fbk_ = clamp(feedback / 64.0f, -1.0f, 1.0f) * 0.85f;
    }
    bool active() const { return mix_ > 0.0f; }

    void process(float& l, float& r)
    {
        phase_ += inc_; if (phase_ >= 1.0f) phase_ -= 1.0f;
        const float m0 = mpc::dsp::WavetableSine::lookup(phase_);
        float m1p = phase_ + 0.25f; if (m1p >= 1.0f) m1p -= 1.0f;
        const float m1 = mpc::dsp::WavetableSine::lookup(m1p);
        float in[2] = { l, r };
        float wet[2];
        if (type_ == 0)
        {
            const float base = 0.007f * sr_, swing = 0.006f * sr_ * depth_;
            const float mods[2] = { m0, m1 };
            for (int c = 0; c < 2; ++c)
            {
                const float d = base + swing * (0.5f + 0.5f * mods[c]) + 1.0f;
                wet[c] = dl_[c].readLinear(d);
                dl_[c].write(in[c] + wet[c] * fbk_);
            }
        }
        else
        {
            const float mods[2] = { m0, m1 };
            for (int c = 0; c < 2; ++c)
            {
                // Sweep 4 first-order allpasses between ~200 Hz and ~4 kHz.
                const float hz = 200.0f * std::exp2((0.5f + 0.5f * mods[c]) * (1.0f + 3.3f * depth_));
                const float tn = std::tan(kPi * std::fmin(hz, 0.45f * sr_) / sr_);
                const float k = (tn - 1.0f) / (tn + 1.0f);
                float x = in[c] + fb_[c] * fbk_;
                for (int s = 0; s < 4; ++s)
                {
                    const float y = k * x + ap_[c][s];
                    ap_[c][s] = x - k * y;
                    x = y;
                }
                fb_[c] = x;
                wet[c] = x;
            }
        }
        l = l * (1.0f - 0.5f * mix_) + wet[0] * mix_;
        r = r * (1.0f - 0.5f * mix_) + wet[1] * mix_;
    }

private:
    mpc::dsp::DelayLine dl_[2];
    float sr_ = 44100.0f, phase_ = 0.0f, inc_ = 0.0f, mix_ = 0.0f, depth_ = 0.0f, fbk_ = 0.0f;
    float ap_[2][4] = {}, fb_[2] = {};
    int type_ = 0;
};

// Stereo (ping-pong capable) delay. Send/feedback/width 0..127.
class StereoDelay
{
public:
    void prepare(double sr)
    {
        sr_ = (float)sr;
        const int maxD = (int)(4.0 * sr) + 4;
        dl_[0].prepare(maxD); dl_[1].prepare(maxD);
        reset();
    }
    void reset() { dl_[0].reset(); dl_[1].reset(); damp_[0] = damp_[1] = 0.0f; }
    void set(float send, float timeMs, float feedback, float width)
    {
        send_ = clamp(send / 127.0f, 0.0f, 1.0f);
        targetSamples_ = clamp(timeMs * 0.001f * sr_, 1.0f, (float)dl_[0].maxDelay());
        fb_ = clamp(feedback / 127.0f, 0.0f, 1.0f) * 0.92f;
        width_ = clamp(width / 127.0f, 0.0f, 1.0f);
    }
    bool active() const { return send_ > 0.0f; }
    void process(float& l, float& r)
    {
        // Glide the delay time to avoid clicks when it changes.
        samples_ += (targetSamples_ - samples_) * 0.0005f;
        if (samples_ < 1.0f) samples_ = targetSamples_;
        const float dL = dl_[0].readLinear(samples_);
        const float dR = dl_[1].readLinear(samples_);
        const float in = 0.5f * (l + r);
        // width 0: two mono echoes, width 1: full ping-pong
        const float inL = in + fb_ * (dL * (1.0f - width_) + dR * width_);
        const float inR = in * (1.0f - width_) + fb_ * (dR * (1.0f - width_) + dL * width_);
        damp_[0] += 0.35f * (inL - damp_[0]);
        damp_[1] += 0.35f * (inR - damp_[1]);
        dl_[0].write(damp_[0]);
        dl_[1].write(damp_[1]);
        l += dL * send_;
        r += dR * send_;
    }
private:
    mpc::dsp::DelayLine dl_[2];
    float sr_ = 44100.0f, send_ = 0.0f, targetSamples_ = 1000.0f, samples_ = 0.0f, fb_ = 0.0f, width_ = 0.0f;
    float damp_[2] = {};
};

// Schroeder/Moorer reverb: 4 damped combs + 2 allpasses per channel.
class Reverb
{
public:
    static constexpr int kCombs = 4, kAps = 2;
    void prepare(double sr)
    {
        sr_ = (float)sr;
        const float scale = sr_ / 44100.0f * 1.3f;   // room for the largest type
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kCombs; ++i) comb_[c][i].assign((size_t)((kCombLen[i] + 23 * c) * scale) + 2, 0.0f);
            for (int i = 0; i < kAps; ++i)   ap_[c][i].assign((size_t)((kApLen[i] + 13 * c) * scale) + 2, 0.0f);
        }
        reset();
    }
    void reset()
    {
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kCombs; ++i) { std::fill(comb_[c][i].begin(), comb_[c][i].end(), 0.0f); combPos_[c][i] = 0; combLp_[c][i] = 0.0f; }
            for (int i = 0; i < kAps; ++i)   { std::fill(ap_[c][i].begin(), ap_[c][i].end(), 0.0f); apPos_[c][i] = 0; }
        }
    }
    // type 0..5 (Echo .. Great hall), send/decay 0..127
    void set(int type, float send, float decay)
    {
        static const float kSize[6] = { 0.3f, 0.45f, 0.65f, 0.8f, 1.0f, 1.25f };
        const float size = kSize[clamp(type, 0, 5)];
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kCombs; ++i)
            {
                int len = (int)((kCombLen[i] + 23 * c) * size * sr_ / 44100.0f);
                combLen_[c][i] = clamp(len, 16, (int)comb_[c][i].size() - 1);
            }
            for (int i = 0; i < kAps; ++i)
            {
                int len = (int)((kApLen[i] + 13 * c) * size * sr_ / 44100.0f);
                apLen_[c][i] = clamp(len, 8, (int)ap_[c][i].size() - 1);
            }
        }
        send_ = clamp(send / 127.0f, 0.0f, 1.0f);
        fb_ = 0.62f + 0.355f * clamp(decay / 127.0f, 0.0f, 1.0f);
    }
    bool active() const { return send_ > 0.0f; }
    void process(float& l, float& r)
    {
        const float in = (l + r) * 0.5f * 0.25f;
        float out[2];
        for (int c = 0; c < 2; ++c)
        {
            float acc = 0.0f;
            for (int i = 0; i < kCombs; ++i)
            {
                std::vector<float>& buf = comb_[c][i];
                int& pos = combPos_[c][i];
                const float y = buf[(size_t)pos];
                combLp_[c][i] += 0.4f * (y - combLp_[c][i]);
                buf[(size_t)pos] = in + combLp_[c][i] * fb_;
                if (++pos >= combLen_[c][i]) pos = 0;
                acc += y;
            }
            for (int i = 0; i < kAps; ++i)
            {
                std::vector<float>& buf = ap_[c][i];
                int& pos = apPos_[c][i];
                const float b = buf[(size_t)pos];
                const float y = b - 0.5f * acc;
                buf[(size_t)pos] = acc + 0.5f * b;
                if (++pos >= apLen_[c][i]) pos = 0;
                acc = y;
            }
            out[c] = acc;
        }
        l += out[0] * send_;
        r += out[1] * send_;
    }
private:
    static constexpr int kCombLen[kCombs] = { 1116, 1277, 1422, 1557 };
    static constexpr int kApLen[kAps] = { 556, 341 };
    std::vector<float> comb_[2][kCombs], ap_[2][kAps];
    int combPos_[2][kCombs] = {}, combLen_[2][kCombs] = {}, apPos_[2][kAps] = {}, apLen_[2][kAps] = {};
    float combLp_[2][kCombs] = {};
    float sr_ = 44100.0f, send_ = 0.0f, fb_ = 0.8f;
};

// Transparent stereo peak limiter with 1.5 ms look-ahead. It turns the level down
// before a peak arrives instead of bending the waveform, so loud settings (Mix Level,
// Volume, big unison chords) get louder without crackling. Ceiling -0.7 dBFS.
class PeakLimiter
{
public:
    static constexpr int kLook = 64;
    void prepare(double sr)
    {
        const float fs = (float)sr;
        att_ = 1.0f - std::exp(-1.0f / (0.0002f * fs));
        rel_ = 1.0f - std::exp(-1.0f / (0.120f * fs));
        hold_ = std::exp(-1.0f / (0.0015f * fs));
        reset();
    }
    void reset()
    {
        for (int i = 0; i < kLook; ++i) bufL_[i] = bufR_[i] = 0.0f;
        pos_ = 0; holdLeft_ = 0; env_ = 0.0f; gain_ = 1.0f;
    }
    void process(float& l, float& r)
    {
        const float peak = std::fmax(std::fabs(l), std::fabs(r));
        // Hold each peak for the whole look-ahead window, so the gain is still down
        // when that peak leaves the delay line; only then let the envelope fall.
        if (peak >= env_) { env_ = peak; holdLeft_ = kLook + 2; }
        else if (holdLeft_ > 0) --holdLeft_;
        else env_ = std::fmax(peak, env_ * hold_);
        const float target = env_ > kCeiling ? kCeiling / env_ : 1.0f;
        gain_ += (target - gain_) * (target < gain_ ? att_ : rel_);
        const float dl = bufL_[pos_], dr = bufR_[pos_];
        bufL_[pos_] = l; bufR_[pos_] = r;
        if (++pos_ >= kLook) pos_ = 0;
        l = clamp(dl * gain_, -0.999f, 0.999f);
        r = clamp(dr * gain_, -0.999f, 0.999f);
    }
private:
    static constexpr float kCeiling = 0.92f;
    float bufL_[kLook] = {}, bufR_[kLook] = {};
    int pos_ = 0, holdLeft_ = 0;
    float env_ = 0.0f, gain_ = 1.0f, att_ = 0.1f, rel_ = 0.001f, hold_ = 0.99f;
};

// Analog-style ADSR. Attack rises along a slightly convex curve and reaches the top in
// `attackMs`; decay and release fall exponentially with time constants `decayTau` /
// `releaseTau` (ms), like an RC envelope. Re-triggering starts from the current level
// (no click), and attack/release never go below a couple of milliseconds.
class Env
{
public:
    void setSampleRate(double sr) { sr_ = (float)sr; }
    void set(float attackMs, float decayTau, float sustain, float releaseTau)
    {
        attackMs = std::fmax(attackMs, 1.0f);
        decayTau = std::fmax(decayTau, 0.3f);
        releaseTau = std::fmax(releaseTau, 1.5f);
        // attack: level += (1.3 - level) * a, reaching 1.0 after attackMs from 0
        attackCoef_ = 1.0f - std::exp(-1.4663f / (attackMs * 0.001f * sr_));
        decayCoef_ = std::exp(-1.0f / (decayTau * 0.001f * sr_));
        releaseCoef_ = std::exp(-1.0f / (releaseTau * 0.001f * sr_));
        sustain_ = clamp(sustain, 0.0f, 1.0f);
    }
    void noteOn() { stage_ = 1; }
    void noteOff() { if (stage_ != 0) stage_ = 4; }
    void reset() { stage_ = 0; level_ = 0.0f; }
    bool isActive() const { return stage_ != 0; }
    float level() const { return level_; }
    float process()
    {
        switch (stage_)
        {
            case 1: // attack
                level_ += (1.3f - level_) * attackCoef_;
                if (level_ >= 1.0f) { level_ = 1.0f; stage_ = 2; }
                break;
            case 2: // decay towards sustain
                level_ = sustain_ + (level_ - sustain_) * decayCoef_;
                if (level_ - sustain_ < 1e-4f) { level_ = sustain_; stage_ = 3; }
                break;
            case 3: // sustain
                level_ = sustain_;
                if (sustain_ <= 0.0f) { level_ = 0.0f; stage_ = 0; }
                break;
            case 4: // release
                level_ *= releaseCoef_;
                if (level_ < 1e-4f) { level_ = 0.0f; stage_ = 0; }
                break;
            default: break;
        }
        return level_;
    }
private:
    float sr_ = 44100.0f, attackCoef_ = 0.1f, decayCoef_ = 0.999f, releaseCoef_ = 0.999f, sustain_ = 1.0f, level_ = 0.0f;
    int stage_ = 0;
};

// Envelope knob (0..127) to time. Attack: 1 ms .. ~16 s to the top. Decay / release time
// constant: 0.5 ms .. ~8 s (the sound is ~-60 dB after about 7 time constants).
inline float envAttackMs(float v) { return 1.0f * std::exp2(clamp(v, 0.0f, 127.0f) * (14.0f / 127.0f)); }
inline float envTauMs(float v)    { return 0.5f * std::exp2(clamp(v, 0.0f, 127.0f) * (14.0f / 127.0f)); }

// Overdrive: tanh saturation that leaves a mid-level signal (0.5) at the same level, so
// turning it up adds grit and compression instead of a big jump in volume.
struct Overdrive
{
    float g = 1.0f, norm = 1.0f;
    bool on = false;
    void set(float amount)   // 0..127
    {
        const float a = clamp(amount, 0.0f, 127.0f) / 127.0f;
        on = a > 0.0f;
        g = 1.0f + 15.0f * a * a;
        norm = 0.5f / std::tanh(0.5f * g);
    }
    float process(float x) const { return on ? fastTanh(g * x) * norm : x; }
};

// State-variable filter (Andrew Simper's TPT form, public technique) with a soft
// saturator on the band-pass state. That keeps it clean at normal settings, lets the
// resonance get thick and "squelchy", and lets it self-oscillate (sing a sine at the
// cutoff) when the damping goes to zero or slightly below at full resonance.
class ResoSVF
{
public:
    enum Mode { kLP = 0, kBP = 1, kHP = 2 };
    void setSampleRate(double sr) { sr_ = (float)sr; }
    void reset() { ic1_ = ic2_ = 0.0f; }
    // k = damping: 2 = very soft, 1.414 = Butterworth, 0 = self-oscillation edge.
    void set(float hz, float k)
    {
        hz = clamp(hz, 10.0f, 0.45f * sr_);
        const float g = std::tan(kPi * hz / sr_);
        k_ = k;
        a1_ = 1.0f / (1.0f + g * (g + k));
        a2_ = g * a1_;
        a3_ = g * a2_;
    }
    float process(float x, int mode)
    {
        const float v3 = x - ic2_;
        const float v1 = a1_ * ic1_ + a2_ * v3;
        const float v2 = ic2_ + a2_ * ic1_ + a3_ * v3;
        ic1_ = mpc::dsp::flushDenormal(sat(2.0f * v1 - ic1_));
        ic2_ = mpc::dsp::flushDenormal(2.0f * v2 - ic2_);
        if (mode == kBP) return v1;
        if (mode == kHP) return x - k_ * v1 - v2;
        return v2;
    }
private:
    // Transparent below ~0.5, rounds off towards +-2.
    static float sat(float s) { return 2.0f * fastTanh(0.5f * s); }
    float sr_ = 44100.0f, k_ = 1.414f, a1_ = 1.0f, a2_ = 0.0f, a3_ = 0.0f, ic1_ = 0.0f, ic2_ = 0.0f;
};

} // namespace jamiro

#endif // JAMIRO_DSP_H
