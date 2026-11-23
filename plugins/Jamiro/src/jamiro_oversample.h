// jamiro_oversample.h - 4x oversampling for the Overdrive and Distortion saturators.
//
// Why: a tanh pushed 25x-40x makes harmonics far above 22 kHz. At the plain sample rate
// they fold back (aliasing) as metallic, out-of-tune tones in the treble. Running the
// saturator at 4x the rate and filtering before coming back removes them.
//
// How: two cascaded 2x stages of polyphase IIR half-band filters (two chains of first-order
// allpass sections). Stage 1 (6 coefficients, steep, ~-80 dB, flat to ~20 kHz) does the
// real work; stage 2 (3 coefficients, ~-90 dB) only has to remove the images of stage 1.
// Method after Laurent de Soras' HIIR library (WTFPL licence); the coefficients were
// computed for this plugin with that design procedure. About 50 multiplies per sample
// plus four saturator calls; no allocation, no tables.

#ifndef JAMIRO_OVERSAMPLE_H
#define JAMIRO_OVERSAMPLE_H

namespace jamiro {

// Stage 1: 6 coefficients, transition band 0.05 (passband to ~0.45 x base rate).
static constexpr float kHb1[6] = {0.060297391f, 0.215971445f, 0.412590720f,
                                  0.604358626f, 0.772715654f, 0.923886139f};
// Stage 2: 3 coefficients, transition band 0.255.
static constexpr float kHb2[3] = {0.069335047f, 0.282591983f, 0.682403397f};

// One set of allpass states. Even coefficients filter path a, odd ones path b.
template <int NC>
struct HalfBandState
{
    float x[NC] = {}, y[NC] = {};
    void reset() { for (int i = 0; i < NC; ++i) x[i] = y[i] = 0.0f; }
    void prime(float v) { for (int i = 0; i < NC; ++i) x[i] = y[i] = v; }
    inline void run(float& a, float& b, const float* c)
    {
        for (int i = 0; i < NC; ++i)
        {
            float& s = (i & 1) ? b : a;
            const float t = (s - y[i]) * c[i] + x[i];
            x[i] = s; y[i] = t; s = t;
        }
    }
};

class Oversampler4x
{
public:
    void reset() { up1_.reset(); up2_.reset(); dn2_.reset(); dn1_.reset(); }
    // Start from a steady signal level instead of silence, so switching the effect on in
    // the middle of a note does not click (every allpass passes a steady level unchanged).
    void prime(float v) { up1_.prime(v); up2_.prime(v); dn2_.prime(v); dn1_.prime(v); }

    // Upsample one sample to four, apply shaper() to each, and return one sample.
    template <class Shaper>
    inline float process(float in, Shaper&& shaper)
    {
        float a = in, b = in;
        up1_.run(a, b, kHb1);                 // 2x: a then b
        float s0 = a, s1 = a, s2 = b, s3 = b;
        up2_.run(s0, s1, kHb2);               // 4x
        up2_.run(s2, s3, kHb2);
        s0 = shaper(s0); s1 = shaper(s1); s2 = shaper(s2); s3 = shaper(s3);
        dn2_.run(s1, s0, kHb2);               // back to 2x
        const float d0 = 0.5f * (s0 + s1);
        dn2_.run(s3, s2, kHb2);
        const float d1 = 0.5f * (s2 + s3);
        float p = d1, q = d0;                 // back to 1x
        dn1_.run(p, q, kHb1);
        return 0.5f * (p + q);
    }

private:
    HalfBandState<6> up1_, dn1_;
    HalfBandState<3> up2_, dn2_;
};

} // namespace jamiro

#endif // JAMIRO_OVERSAMPLE_H
