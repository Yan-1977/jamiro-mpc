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

#include "jamiro_lfo_shapes.h"
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
inline float lfoHz(float v) { float hz = 0.0621f * std::exp2(clamp(v, 0.0f, 127.0f) * 0.0976f); return hz > 120.0f ? 120.0f : hz; }

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
        static const float kFade[kTables] = { 0.28f, 2.0f, 0.9f, 0.12f, 1.4f, 2.5f, 1.0f, 1.2f, 0.5f };
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
    // Harmonic levels 1..80 of each table, measured on a Novation X-Station (osc 1 alone, filter
    // open, note C4) and expressed relative to the strongest harmonic. Only the levels are used:
    // the phases are Jamiro's own, so the waveforms are resynthesised, not copied. The hardware
    // waves are static while a key is held, so attack and body are the same spectrum.
    static constexpr int kMeasured = 80;
    static double amp(int table, bool attack, int n)
    {
        (void)attack;
        static const float kSpec[kTables][kMeasured] = {
        { 1.0f, 0.2329f, 0.3766f, 0.0076f, 0.0211f, 0.6227f, 0.0291f, 0.0146f, 0.0258f, 0.0088f, 0.005f, 0.0039f, 0.0027f, 0.0049f, 0.0061f, 0.0017f, 0.0039f, 0.0255f, 0.0032f, 0.0f, 0.0016f, 0.0f, 0.0009f, 0.0018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0022f, 0.0012f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, // Organ
        { 0.1317f, 1.0f, 0.8136f, 0.9742f, 0.1719f, 0.5691f, 0.2008f, 0.4486f, 0.0448f, 0.0803f, 0.0258f, 0.0363f, 0.0854f, 0.0837f, 0.0534f, 0.0826f, 0.0444f, 0.0343f, 0.0325f, 0.0185f, 0.0277f, 0.0075f, 0.0053f, 0.007f, 0.0192f, 0.0199f, 0.0142f, 0.0312f, 0.032f, 0.0104f, 0.0188f, 0.005f, 0.0172f, 0.0111f, 0.0045f, 0.0103f, 0.0018f, 0.0029f, 0.0013f, 0.0071f, 0.0036f, 0.0062f, 0.005f, 0.0034f, 0.0018f, 0.0011f, 0.0017f, 0.001f, 0.0022f, 0.0015f, 0.0019f, 0.0009f, 0.0014f, 0.0011f, 0.0012f, 0.0009f, 0.0015f, 0.001f, 0.001f, 0.0011f, 0.0f, 0.0f, 0.002f, 0.0009f, 0.0023f, 0.0032f, 0.0014f, 0.003f, 0.0f, 0.0022f, 0.0035f, 0.0009f, 0.0012f, 0.0013f, 0.001f, 0.0009f, 0.0f, 0.0011f, 0.0041f, 0.0035f }, // Harpsichord
        { 1.0f, 0.1431f, 0.0395f, 0.0024f, 0.0015f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0278f, 0.0009f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, // E.Piano
        { 1.0f, 0.3579f, 0.1149f, 0.0354f, 0.0149f, 0.0084f, 0.0071f, 0.0033f, 0.0309f, 0.0369f, 0.0093f, 0.0126f, 0.0121f, 0.034f, 0.0334f, 0.0155f, 0.0112f, 0.0027f, 0.0031f, 0.0056f, 0.0052f, 0.0125f, 0.03f, 0.0172f, 0.0081f, 0.0174f, 0.0142f, 0.0429f, 0.0433f, 0.0139f, 0.0065f, 0.0048f, 0.0027f, 0.0f, 0.0021f, 0.0f, 0.0039f, 0.002f, 0.0f, 0.0036f, 0.0031f, 0.003f, 0.0013f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, // Slap
        { 0.3178f, 1.0f, 0.5627f, 0.4186f, 0.3769f, 0.0761f, 0.176f, 0.0039f, 0.0728f, 0.0167f, 0.0257f, 0.0133f, 0.0085f, 0.0076f, 0.0022f, 0.0039f, 0.0f, 0.0023f, 0.0f, 0.0017f, 0.0012f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, // Rhodes
        { 0.1122f, 1.0f, 0.0278f, 0.6846f, 0.0189f, 0.1497f, 0.0186f, 0.1658f, 0.0143f, 0.149f, 0.022f, 0.132f, 0.0265f, 0.187f, 0.024f, 0.0482f, 0.0293f, 0.1421f, 0.02f, 0.0055f, 0.0246f, 0.0755f, 0.0101f, 0.0154f, 0.013f, 0.0306f, 0.0062f, 0.0095f, 0.0049f, 0.0089f, 0.004f, 0.0048f, 0.0008f, 0.0034f, 0.003f, 0.0023f, 0.0029f, 0.0027f, 0.0098f, 0.0041f, 0.0043f, 0.0032f, 0.01f, 0.0f, 0.0033f, 0.0014f, 0.0042f, 0.0f, 0.002f, 0.0f, 0.0014f, 0.0f, 0.0009f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f }, // Tine
        { 0.0444f, 0.7424f, 0.5824f, 0.8546f, 1.0f, 0.3892f, 0.8869f, 0.0377f, 0.815f, 0.2481f, 0.5523f, 0.3698f, 0.3347f, 0.3801f, 0.1746f, 0.3451f, 0.0688f, 0.293f, 0.0081f, 0.2396f, 0.0436f, 0.194f, 0.0727f, 0.1507f, 0.0887f, 0.1059f, 0.088f, 0.0702f, 0.0869f, 0.0358f, 0.0785f, 0.0114f, 0.0658f, 0.0077f, 0.0491f, 0.0152f, 0.0338f, 0.0183f, 0.0212f, 0.0178f, 0.0123f, 0.0149f, 0.0063f, 0.011f, 0.0027f, 0.0076f, 0.0f, 0.0049f, 0.0f, 0.0028f, 0.0f, 0.0013f, 0.001f, 0.0f, 0.0012f, 0.0f, 0.0016f, 0.0f, 0.0019f, 0.0009f, 0.0019f, 0.0012f, 0.0018f, 0.0014f, 0.0016f, 0.0015f, 0.0014f, 0.0015f, 0.0011f, 0.0015f, 0.0008f, 0.0015f, 0.0f, 0.0014f, 0.0f, 0.0012f, 0.0f, 0.0011f, 0.0f, 0.001f }, // Wurly
        { 0.0336f, 1.0f, 0.3616f, 0.6493f, 0.3356f, 0.4474f, 0.2731f, 0.3402f, 0.2873f, 0.0753f, 0.3128f, 0.2136f, 0.2862f, 0.0781f, 0.2812f, 0.1376f, 0.1431f, 0.0519f, 0.0298f, 0.0045f, 0.0152f, 0.0196f, 0.0053f, 0.0206f, 0.0253f, 0.0376f, 0.0234f, 0.0391f, 0.0223f, 0.0185f, 0.0311f, 0.028f, 0.0212f, 0.0121f, 0.0047f, 0.0061f, 0.0071f, 0.0039f, 0.0115f, 0.009f, 0.0107f, 0.0034f, 0.0081f, 0.0024f, 0.0023f, 0.0012f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0009f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0012f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, // Clav
        { 1.0f, 0.765f, 0.4788f, 0.2845f, 0.1963f, 0.1469f, 0.1162f, 0.0953f, 0.0801f, 0.0691f, 0.0597f, 0.0528f, 0.0471f, 0.0422f, 0.0381f, 0.0345f, 0.0314f, 0.0289f, 0.0266f, 0.0246f, 0.0231f, 0.0213f, 0.0201f, 0.0192f, 0.0183f, 0.0173f, 0.0166f, 0.0159f, 0.0154f, 0.0149f, 0.0144f, 0.014f, 0.0137f, 0.0134f, 0.0129f, 0.0127f, 0.0124f, 0.0121f, 0.0118f, 0.0115f, 0.0113f, 0.011f, 0.0107f, 0.0105f, 0.0105f, 0.0099f, 0.0097f, 0.0094f, 0.0092f, 0.0089f, 0.0087f, 0.0084f, 0.0082f, 0.008f, 0.0077f, 0.0075f, 0.0073f, 0.0071f, 0.0069f, 0.0067f, 0.0065f, 0.0064f, 0.0062f, 0.0061f, 0.0059f, 0.0057f, 0.0056f, 0.0054f, 0.0053f, 0.0052f, 0.0051f, 0.005f, 0.0049f, 0.0048f, 0.0047f, 0.0045f, 0.0044f, 0.0043f, 0.004f, 0.0038f }, // AnaBass
        };
        if (n <= kMeasured) return (double)kSpec[table][n - 1];
        // above the measured range (low notes only): let the top of the spectrum roll off gently
        double tail = 0.0;
        for (int k = kMeasured - 9; k <= kMeasured; ++k) tail += (double)kSpec[table][k - 1];
        tail *= 0.1;
        return tail * std::pow((double)kMeasured / (double)n, 2.0);
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
    float n1 = 0.0f, n2 = 0.0f, n3 = 0.0f;   // noise filter states
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
            case kTriangle: return 1.288f * (1.0f - 4.0f * std::fabs(t - 0.5f));   // +2.2 dB vs saw, as measured on the X-Station
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
            case kWhite: return 1.122f * rng.nextBipolar();
            // Coloured noises, shaped and levelled to match X-Station measurements (one-pole stages at 44.1 kHz).
            case kHPNoise:   // white plus a high shelf (+13 dB at 16 kHz)
            { const float x = rng.nextBipolar(); n1 += 0.68012f * (x - n1); return 0.3003f * (x + 8.0f * (x - n1)); }
            case kBPNoise:   // band around 2-5 kHz, about 27 dB above the low end
            {
                const float x = rng.nextBipolar();
                n1 += 0.22621f * (x - n1); const float h1 = x - n1;
                n2 += 0.22621f * (h1 - n2); const float h2 = h1 - n2;
                n3 += 0.34782f * (h2 - n3);
                return 3.073f * (n3 + 0.03f * x);
            }
            case kHBPNoise:  // white plus a steep lift above 6 kHz
            {
                const float x = rng.nextBipolar();
                n1 += 0.57466f * (x - n1); const float h1 = x - n1;
                n2 += 0.57466f * (h1 - n2); const float h2 = h1 - n2;
                return 0.4857f * (x + 8.0f * h2);
            }
            default:
            {
                // level of each table relative to the saw, as on the X-Station
                static const float kTableGain[9] = { 1.413f, 1.259f, 1.462f, 1.318f, 1.318f, 1.288f, 0.912f, 1.148f, 1.109f };
                const int k = wave - kFirstTable;
                return kTableGain[k] * tables.read(k, t, tableLevel, bright);
            }
        }
    }
};

// ---------------------------------------------------------------------------
// LFO waveforms (index == "LFO Wave" choice). Evaluated at control rate.
// ---------------------------------------------------------------------------
// The 32 shapes of the X-Station. The first 10 keep the positions they had in earlier Jamiro
// versions (so saved sounds keep their LFO); the other 22 follow. Shapes were measured on the
// hardware (see jamiro_lfo_shapes.h); all are bipolar -1..1, "Unipolar" maps them to 0..1.
enum LfoWave
{
    kLSine = 0, kLTri, kLSaw, kLSquare, kLSH, kLQuantSH, kLCrossfade, kLExpDec1, kLAttExp1, kLChromatic,
    kLExpDec2, kLExpDec3, kLAttExp2, kLAttExp3, kLSustExp1, kLSustExp2, kLSustExp3,
    kLPiano1, kLPiano2, kLPiano3, kLExpUpDown, kLMajModes, kLMajor7,
    kLPattern1, kLPattern2, kLPattern3, kLPattern4, kLPattern5, kLPattern6, kLPattern7, kLPattern8, kLPattern9,
    kNumLfoWaves
};

inline std::vector<std::string> lfoWaveNames()
{
    return { "Sine", "Triangle", "Saw", "Square", "Random S/H", "Quant S/H", "Crossfade", "Exp Dec 1", "Att Exp 1", "Chromatic",
             "Exp Dec 2", "Exp Dec 3", "Att Exp 2", "Att Exp 3", "Sust Exp 1", "Sust Exp 2", "Sust Exp 3",
             "Piano Env 1", "Piano Env 2", "Piano Env 3", "Exp Up Down", "Maj-Modes", "Major-7",
             "Pattern 1", "Pattern 2", "Pattern 3", "Pattern 4", "Pattern 5", "Pattern 6", "Pattern 7", "Pattern 8", "Pattern 9" };
}

// X-Station LFO number (0..31, the order of its manual) -> Jamiro LFO wave.
inline int lfoFromXStation(int x)
{
    static const int kMap[32] = { kLSine, kLTri, kLSaw, kLSquare, kLSH, kLQuantSH, kLCrossfade,
                                  kLExpDec1, kLExpDec2, kLExpDec3, kLAttExp1, kLAttExp2, kLAttExp3,
                                  kLSustExp1, kLSustExp2, kLSustExp3, kLPiano1, kLPiano2, kLPiano3, kLExpUpDown,
                                  kLChromatic, kLMajModes, kLMajor7,
                                  kLPattern1, kLPattern2, kLPattern3, kLPattern4, kLPattern5, kLPattern6, kLPattern7, kLPattern8, kLPattern9 };
    return kMap[clamp(x, 0, 31)];
}

struct VoiceLfo
{
    float phase = 0.0f, held = 0.0f, heldBin = 1.0f, value = 0.0f;
    bool done = false;   // one-shot finished

    // Row of kLfoShapes for the measured smooth shapes, -1 for the others.
    static int measuredRow(int wave)
    {
        switch (wave)
        {
            case kLCrossfade: return 0; case kLExpDec1: return 1; case kLExpDec2: return 2; case kLExpDec3: return 3;
            case kLAttExp1: return 4; case kLAttExp2: return 5; case kLAttExp3: return 6;
            case kLSustExp1: return 7; case kLSustExp2: return 8; case kLSustExp3: return 9;
            case kLPiano1: return 10; case kLPiano2: return 11; case kLPiano3: return 12; case kLExpUpDown: return 13;
            default: return -1;
        }
    }

    // Unipolar value 0..1 of the stepped / table shapes at phase t.
    static float shapeUni(int wave, float t)
    {
        const int row = measuredRow(wave);
        if (row >= 0)
        {
            const float x = t * (float)kLfoShapePoints;
            int i = (int)x; const float f = x - (float)i;
            i = clamp(i, 0, kLfoShapePoints - 1);
            const int j = i + 1 < kLfoShapePoints ? i + 1 : 0;
            return kLfoShapes[row][i] + (kLfoShapes[row][j] - kLfoShapes[row][i]) * f;
        }
        switch (wave)
        {
            case kLChromatic: return std::floor(t * 12.0f) * (1.0f / 12.0f);      // 0..11 semitones at depth 30
            case kLMajModes: { static const float k[8] = {0, 2, 4, 5, 7, 9, 11, 12}; return k[clamp((int)(t * 8.0f), 0, 7)] / 12.0f; }
            case kLMajor7:   { static const float k[4] = {0, 4, 7, 11}; return k[clamp((int)(t * 4.0f), 0, 3)] / 12.0f; }
            case kLPattern1: { static const float k[4] = {0, 3, 7, 11}; return k[clamp((int)(t * 4.0f), 0, 3)] / 12.0f; }
            default:
            {
                // Patterns 2-9: 16-step rhythms between the note (0) and its octave (1) at depth 30.
                static const char* kRhythm[8] = { "XXXX....XX..XX..", "XX..XX..XXXX....", "XXXXXX..XX.XX.X.", "XX.XX.X.XXXXXX..",
                                                  "XXXXXXXX..XXXXX.", "X.X.X.X.XXX.XXX.", "X.X.XX.XXXXXXXX.", "XXX.XXX.X..X..X." };
                const int pat = clamp(wave - kLPattern2, 0, 7);
                return kRhythm[pat][clamp((int)(t * 16.0f), 0, 15)] == 'X' ? 1.0f : 0.0f;
            }
        }
    }

    // Advance by `cycles` and return the new value (-1..1).
    float tick(int wave, float cycles, bool oneShot, mpc::dsp::Xorshift32& rng)
    {
        if (!done) phase += cycles;
        if (phase >= 1.0f)
        {
            if (oneShot) { phase = 0.99999f; done = true; }
            else
            {
                phase -= std::floor(phase);
                held = rng.nextBipolar();
                heldBin = rng.nextBipolar() >= 0.0f ? 1.0f : -1.0f;
            }
        }
        const float t = phase;
        switch (wave)
        {
            case kLSine:    value = mpc::dsp::WavetableSine::lookup(t); break;          // starts in the middle, rising
            case kLTri:     value = t < 0.5f ? 4.0f * t - 1.0f : 3.0f - 4.0f * t; break; // starts at the bottom, rising
            case kLSaw:     value = 1.0f - 2.0f * t; break;                              // falling, as on the X-Station
            case kLSquare:  value = t < 0.5f ? 1.0f : -1.0f; break;
            case kLSH:      value = held; break;
            case kLQuantSH: value = heldBin; break;                                      // random top or bottom each cycle
            default:        value = 2.0f * shapeUni(wave, t) - 1.0f; break;
        }
        return value;
    }
    void restart(float ph) { phase = wrapPhase(ph); done = false; }
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

// Small ring buffer with 4-point Hermite interpolation (smoother than linear for
// modulated delays: no dulling, no zipper on chorus / delay time changes).
struct HermiteDelay
{
    std::vector<float> buf;
    int mask = 0, w = 0;
    void prepare(int maxSamples)
    {
        int n = 16;
        while (n < maxSamples + 8) n <<= 1;
        buf.assign((size_t)n, 0.0f);
        mask = n - 1; w = 0;
    }
    void reset() { std::fill(buf.begin(), buf.end(), 0.0f); w = 0; }
    void write(float x) { buf[(size_t)w] = x; w = (w + 1) & mask; }
    int maxDelay() const { return mask - 4; }
    float read(float d) const   // d samples ago, d >= 1
    {
        d = clamp(d, 1.0f, (float)(mask - 4));
        const int i = (int)d;
        const float f = d - (float)i;
        const float xm1 = buf[(size_t)((w - i + 1) & mask)];
        const float x0 = buf[(size_t)((w - i) & mask)];
        const float x1 = buf[(size_t)((w - i - 1) & mask)];
        const float x2 = buf[(size_t)((w - i - 2) & mask)];
        const float c1 = 0.5f * (x1 - xm1);
        const float c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
        const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
        return ((c3 * f + c2) * f + c1) * f + x0;
    }
};

// Stereo chorus (3 voices per side) / phaser (6 stages per side).
class ChorusPhaser
{
public:
    void prepare(double sr)
    {
        sr_ = (float)sr;
        dl_[0].prepare((int)(0.04 * sr) + 8);
        dl_[1].prepare((int)(0.04 * sr) + 8);
        reset();
    }
    void reset()
    {
        dl_[0].reset(); dl_[1].reset();
        for (int c = 0; c < 2; ++c) { fb_[c] = 0.0f; hp_[c] = 0.0f; lp_[c] = 0.0f; for (int s = 0; s < kStages; ++s) ap_[c][s] = 0.0f; k_[c] = 0.0f; }
        phase_ = 0.0f; ctr_ = 0;
        for (int c = 0; c < 2; ++c) for (int v = 0; v < 3; ++v) { tap_[c][v] = 0.0f; tapStep_[c][v] = 0.0f; }
    }
    // type 0 = chorus, 1 = phaser. mix/rate/depth 0..127, feedback -64..63
    void set(int type, float mix, float rate, float depth, float feedback)
    {
        type_ = type;
        mix_ = clamp(mix / 127.0f, 0.0f, 1.0f);
        inc_ = 0.05f * std::exp2(clamp(rate, 0.0f, 127.0f) / 127.0f * 8.0f) / sr_;
        depth_ = clamp(depth / 127.0f, 0.0f, 1.0f);
        fbk_ = clamp(feedback / 64.0f, -1.0f, 1.0f) * (type == 0 ? 0.6f : 0.85f);
        hpA_ = 1.0f - std::exp(-kTwoPi * 150.0f / sr_);
        lpA_ = 1.0f - std::exp(-kTwoPi * 7000.0f / sr_);
    }
    bool active() const { return mix_ > 0.0f; }

    void process(float& l, float& r)
    {
        phase_ += inc_; if (phase_ >= 1.0f) phase_ -= 1.0f;
        float in[2] = { l, r }, wet[2];
        if (type_ == 0)
        {
            // Three taps per side, 120 degrees apart, the right side a quarter turn later.
            // The LFO is evaluated every 8 samples; tap delays ramp linearly in between.
            if ((ctr_++ & 7) == 0)
            {
                const float base = 0.0075f * sr_, swing = 0.0055f * sr_ * (0.15f + 0.85f * depth_);
                float ahead = phase_ + 8.0f * inc_; if (ahead >= 1.0f) ahead -= 1.0f;
                for (int c = 0; c < 2; ++c)
                    for (int v = 0; v < 3; ++v)
                    {
                        float p = ahead + (float)v / 3.0f + (c ? 0.25f : 0.0f);
                        p -= std::floor(p);
                        const float target = base * (1.0f + 0.35f * (float)v) + swing * (0.5f + 0.5f * mpc::dsp::WavetableSine::lookup(p));
                        if (tap_[c][v] <= 0.0f) tap_[c][v] = target;
                        tapStep_[c][v] = (target - tap_[c][v]) * 0.125f;
                    }
            }
            for (int c = 0; c < 2; ++c)
            {
                float sum = 0.0f;
                for (int v = 0; v < 3; ++v) { tap_[c][v] += tapStep_[c][v]; sum += dl_[c].read(tap_[c][v]); }
                sum *= (1.0f / 3.0f);
                lp_[c] += lpA_ * (sum - lp_[c]);            // tame the feedback brightness
                dl_[c].write(in[c] + lp_[c] * fbk_);
                hp_[c] += hpA_ * (sum - hp_[c]);            // keep the lows out of the wet signal
                wet[c] = sum - hp_[c];
            }
            l = l * (1.0f - 0.45f * mix_) + wet[0] * mix_ * 0.95f;
            r = r * (1.0f - 0.45f * mix_) + wet[1] * mix_ * 0.95f;
        }
        else
        {
            if ((ctr_++ & 7) == 0)
                for (int c = 0; c < 2; ++c)
                {
                    float p = phase_ + (c ? 0.25f : 0.0f); if (p >= 1.0f) p -= 1.0f;
                    const float m = 0.5f + 0.5f * mpc::dsp::WavetableSine::lookup(p);
                    // exponential sweep from 120 Hz up to ~120 Hz x 2^(1..6.5 octaves)
                    const float hz = std::fmin(120.0f * std::exp2(m * (1.0f + 5.5f * depth_)), 0.42f * sr_);
                    const float tn = std::tan(kPi * hz / sr_);
                    k_[c] = (tn - 1.0f) / (tn + 1.0f);
                }
            for (int c = 0; c < 2; ++c)
            {
                float x = in[c] + fb_[c] * fbk_;
                for (int s = 0; s < kStages; ++s)
                {
                    const float y = k_[c] * x + ap_[c][s];
                    ap_[c][s] = x - k_[c] * y;
                    x = y;
                }
                fb_[c] = mpc::dsp::flushDenormal(x);
                wet[c] = x;
            }
            // Equal dry + wet at full mix gives the deepest notches.
            l = l * (1.0f - 0.5f * mix_) + wet[0] * 0.5f * mix_;
            r = r * (1.0f - 0.5f * mix_) + wet[1] * 0.5f * mix_;
        }
    }

private:
    static constexpr int kStages = 6;
    HermiteDelay dl_[2];
    float sr_ = 44100.0f, phase_ = 0.0f, inc_ = 0.0f, mix_ = 0.0f, depth_ = 0.0f, fbk_ = 0.0f;
    float ap_[2][kStages] = {}, fb_[2] = {}, hp_[2] = {}, lp_[2] = {}, k_[2] = {};
    float hpA_ = 0.02f, lpA_ = 0.6f;
    float tap_[2][3] = {}, tapStep_[2][3] = {};
    int type_ = 0;
    unsigned ctr_ = 0;
};

// Stereo (ping-pong capable) tape-style delay. Send/feedback/width 0..127.
// Repeats are band-limited (low-cut + high-cut) and softly saturated in the loop, so
// they darken like an analog echo and can never run away, even at full feedback.
class StereoDelay
{
public:
    void prepare(double sr)
    {
        sr_ = (float)sr;
        const int maxD = (int)(4.0 * sr) + 8;
        dl_[0].prepare(maxD); dl_[1].prepare(maxD);
        lpA_ = 1.0f - std::exp(-kTwoPi * 5500.0f / sr_);
        hpA_ = 1.0f - std::exp(-kTwoPi * 90.0f / sr_);
        modInc_ = 0.35f / sr_;
        reset();
    }
    void reset()
    {
        dl_[0].reset(); dl_[1].reset();
        for (int c = 0; c < 2; ++c) { lp_[c] = hp_[c] = 0.0f; }
        modPhase_ = 0.0f; samples_ = 0.0f;
    }
    void set(float send, float timeMs, float feedback, float width)
    {
        send_ = clamp(send / 127.0f, 0.0f, 1.0f);
        targetSamples_ = clamp(timeMs * 0.001f * sr_, 2.0f, (float)dl_[0].maxDelay() - 64.0f);
        fb_ = clamp(feedback / 127.0f, 0.0f, 1.0f) * 0.97f;
        width_ = clamp(width / 127.0f, 0.0f, 1.0f);
    }
    bool active() const { return send_ > 0.0f; }
    void process(float& l, float& r)
    {
        // Glide the delay time (tape-like pitch bend instead of clicks) and add a tiny wow.
        if (samples_ < 1.0f) samples_ = targetSamples_;
        samples_ += (targetSamples_ - samples_) * 0.0004f;
        modPhase_ += modInc_; if (modPhase_ >= 1.0f) modPhase_ -= 1.0f;
        const float wow = 0.0003f * sr_ * mpc::dsp::WavetableSine::lookup(modPhase_);
        const float dL = dl_[0].read(samples_ + wow);
        const float dR = dl_[1].read(samples_ - wow);
        const float in = 0.5f * (l + r);
        // width 0: two mono echoes, width 1: full ping-pong
        float fbL = dL * (1.0f - width_) + dR * width_;
        float fbR = dR * (1.0f - width_) + dL * width_;
        float x[2] = { in + fb_ * fbL, in * (1.0f - width_) + fb_ * fbR };
        for (int c = 0; c < 2; ++c)
        {
            lp_[c] += lpA_ * (x[c] - lp_[c]);
            hp_[c] += hpA_ * (lp_[c] - hp_[c]);
            float y = lp_[c] - hp_[c];
            y = fastTanh(y * 0.8f) * 1.25f;   // gentle tape saturation, bounded
            dl_[c].write(mpc::dsp::flushDenormal(y));
        }
        l += dL * send_;
        r += dR * send_;
    }
private:
    HermiteDelay dl_[2];
    float sr_ = 44100.0f, send_ = 0.0f, targetSamples_ = 1000.0f, samples_ = 0.0f, fb_ = 0.0f, width_ = 0.0f;
    float lp_[2] = {}, hp_[2] = {}, lpA_ = 0.5f, hpA_ = 0.01f, modPhase_ = 0.0f, modInc_ = 0.0f;
};

// Stereo reverb: pre-delay, 8 damped feedback combs + 4 series allpasses per side, with
// the classic public-domain "Freeverb" tunings and stereo spread. Six room sizes.
class Reverb
{
public:
    static constexpr int kCombs = 8, kAps = 4;
    void prepare(double sr)
    {
        sr_ = (float)sr;
        const float scale = sr_ / 44100.0f * 1.25f;   // room for the largest type
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kCombs; ++i) comb_[c][i].assign((size_t)((kCombLen[i] + kSpread * c) * scale) + 2, 0.0f);
            for (int i = 0; i < kAps; ++i)   ap_[c][i].assign((size_t)((kApLen[i] + kSpread * c) * scale) + 2, 0.0f);
        }
        pre_.prepare((int)(0.08 * sr) + 8);
        inHpA_ = 1.0f - std::exp(-kTwoPi * 120.0f / sr_);
        inLpA_ = 1.0f - std::exp(-kTwoPi * 9000.0f / sr_);
        reset();
    }
    void reset()
    {
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kCombs; ++i) { std::fill(comb_[c][i].begin(), comb_[c][i].end(), 0.0f); combPos_[c][i] = 0; combLp_[c][i] = 0.0f; }
            for (int i = 0; i < kAps; ++i)   { std::fill(ap_[c][i].begin(), ap_[c][i].end(), 0.0f); apPos_[c][i] = 0; }
        }
        pre_.reset(); inHp_ = inLp_ = 0.0f;
    }
    // type 0..5 (Echo .. Great hall), send/decay 0..127
    void set(int type, float send, float decay)
    {
        static const float kSize[6]   = { 0.32f, 0.48f, 0.68f, 0.82f, 1.0f, 1.2f };
        static const float kPreMs[6]  = { 2.0f, 6.0f, 12.0f, 14.0f, 22.0f, 32.0f };
        static const float kDamp[6]   = { 0.45f, 0.35f, 0.25f, 0.3f, 0.22f, 0.18f };
        const int t = clamp(type, 0, 5);
        const float size = kSize[t];
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < kCombs; ++i)
                combLen_[c][i] = clamp((int)((kCombLen[i] + kSpread * c) * size * sr_ / 44100.0f), 16, (int)comb_[c][i].size() - 1);
            for (int i = 0; i < kAps; ++i)
                apLen_[c][i] = clamp((int)((kApLen[i] + kSpread * c) * size * sr_ / 44100.0f), 8, (int)ap_[c][i].size() - 1);
        }
        send_ = clamp(send / 127.0f, 0.0f, 1.0f);
        fb_ = 0.70f + 0.28f * std::pow(clamp(decay / 127.0f, 0.0f, 1.0f), 0.8f);
        damp_ = kDamp[t];
        preSamples_ = kPreMs[t] * 0.001f * sr_;
    }
    bool active() const { return send_ > 0.0f; }
    void process(float& l, float& r)
    {
        // band-limit the input (no boomy lows, no fizzy highs) and pre-delay it
        float in = (l + r) * 0.5f;
        inLp_ += inLpA_ * (in - inLp_);
        inHp_ += inHpA_ * (inLp_ - inHp_);
        pre_.write(inLp_ - inHp_);
        const float x = pre_.read(std::fmax(preSamples_, 1.0f)) * 0.03f;
        float out[2];
        for (int c = 0; c < 2; ++c)
        {
            float acc = 0.0f;
            for (int i = 0; i < kCombs; ++i)
            {
                std::vector<float>& buf = comb_[c][i];
                int& pos = combPos_[c][i];
                const float y = buf[(size_t)pos];
                combLp_[c][i] = y * (1.0f - damp_) + combLp_[c][i] * damp_;
                buf[(size_t)pos] = mpc::dsp::flushDenormal(x + combLp_[c][i] * fb_);
                if (++pos >= combLen_[c][i]) pos = 0;
                acc += y;
            }
            for (int i = 0; i < kAps; ++i)
            {
                std::vector<float>& buf = ap_[c][i];
                int& pos = apPos_[c][i];
                const float b = buf[(size_t)pos];
                const float y = b - acc;
                buf[(size_t)pos] = mpc::dsp::flushDenormal(acc + 0.5f * b);
                if (++pos >= apLen_[c][i]) pos = 0;
                acc = y;
            }
            out[c] = acc;
        }
        l += out[0] * send_ * 1.6f;
        r += out[1] * send_ * 1.6f;
    }
private:
    static constexpr int kSpread = 23;
    static constexpr int kCombLen[kCombs] = { 1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617 };
    static constexpr int kApLen[kAps] = { 556, 441, 341, 225 };
    std::vector<float> comb_[2][kCombs], ap_[2][kAps];
    int combPos_[2][kCombs] = {}, combLen_[2][kCombs] = {}, apPos_[2][kAps] = {}, apLen_[2][kAps] = {};
    float combLp_[2][kCombs] = {};
    HermiteDelay pre_;
    float sr_ = 44100.0f, send_ = 0.0f, fb_ = 0.84f, damp_ = 0.25f, preSamples_ = 0.0f;
    float inHp_ = 0.0f, inLp_ = 0.0f, inHpA_ = 0.02f, inLpA_ = 0.7f;
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

// Overdrive: drives the FILTER OUTPUT into a tanh saturator, where it adds grit and
// bite even to square waves and to resonant, closed-filter sounds. It is level-matched
// per voice in Jamiro.cpp, so the knob changes the character, not the volume.
struct Overdrive
{
    float gPost = 1.0f;
    bool on = false;
    void set(float amount)   // 0..127
    {
        const float a = clamp(amount, 0.0f, 127.0f) / 127.0f;
        on = a > 0.0f;
        gPost = 1.0f + 24.0f * a * a;
    }
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
