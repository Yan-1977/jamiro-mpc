# Jamiro

A polyphonic synthesizer for Akai MPC standalone hardware (MPC One/One+, Live/Live II/Live III,
X/X SE, Key 37/61, XL, and Force) by **Yann**.

Jamiro recreates the synth engine built into the **Novation X-Station** keyboard: the "KS" engine
shared with the KS4, KS5, KS Rack and K-Station. It uses the same architecture and the same
0..127 / -64..+63 parameter values as the hardware, and it reads **X-Station / KS `.syx` patch
banks**. If you own the original banks (for example Novation's "100 New Sounds" update or the
Jamiroquai bank made for the X-Station), drop them in the plugin's `Content/Banks/` folder and
play your X-Station sounds on the MPC.

> Jamiro is an independent project. It is not affiliated with or endorsed by Novation, Focusrite
> or Jamiroquai. "Novation" and "X-Station" are trademarks of their owners. No Novation sounds,
> samples or firmware are included; all code is original (MIT licence).

![Jamiro on the MPC screen (FILT/ENV page)](docs/jamiro-screen.png)

*Preview of the FILT/ENV page, drawn from the plugin's skin files.*

## Features

- 8 voices, 3 oscillators per voice, plus noise and ring modulation (1×2). Unison up to 8 voices.
- Voice modes Mono, Mono AG (auto-glide on legato notes), Poly 1 and Poly 2; portamento.
- 17 waveforms: sine, triangle, saw (with "double saw"), variable pulse, 4 noise types, and 9
  additive tables in the spirit of the X-Station's sampled waves (organ, harpsichord, e-piano,
  slap bass, Rhodes, tine, Wurly, Clavinet, analog bass). These are resynthesised, not samples.
- Osc 1 > 2 hard sync, osc 2 > 3 FM with its own AD envelope, LFO / envelope level modulation.
- Filter: low-, band- or high-pass, 12 or 24 dB, overdrive, Q normalise, key tracking, envelope,
  LFO 2, velocity, mod wheel and aftertouch.
- Amp and mod envelopes (ADSR + velocity); 2 LFOs (10 shapes, delay, tempo sync, key sync, unipolar).
- Arpeggiator synced to the MPC tempo: Up, Down, Up-Down 1/2, Played, Random, Chord; 1-4 octaves;
  gate; latch.
- Effects: distortion, tilt EQ, chorus/phaser, stereo delay (tempo sync), reverb (6 sizes),
  pan and auto-pan.
- 6 touch-screen pages, all controls on Q-Links. 7 original programs built in.

## Tested on

| Device | Generation | MPC OS | Version |
|---|---|---|---|
| MPC Key 37 | Gen 1 (32-bit, armv7) | 3.10.1 or later | 1.0.0 |

Hardware testing so far is a quick listening test on Gen 1: the plugin loads, the screen pages
show, and the presets play. Editing, automation, the arpeggiator and long sessions have not been
tested on hardware yet. Builds are also made for Gen 2 (aarch64) MPCs but have not been tried on
one. Reports from any MPC are very welcome (open an issue).

## Installing

Requires MPC OS 3.10.1 or later. Enable **Preferences → Enhancements (Beta) → Enable
Enhancements and External Plugin Support**.

Install it from the OpenMPC Plugin Manager, or copy the `Yann - VST - Jamiro` folder from the
release zip into a `VST2 Plugins` folder at the root of a drive (SD card, USB drive or internal disk).

### Loading X-Station banks

Copy your `.syx` files into the plugin's `Content/Banks/` folder:

```
VST2 Plugins/Yann - VST - Jamiro/Content/Banks/1 My Bank.syx
VST2 Plugins/Yann - VST - Jamiro/Content/Banks/2 Another Bank.syx
```

At start-up Jamiro converts every program it finds, in file-name order. The first file becomes
programs 100-199, the second 200-299, and so on. It reads X-Station bank and program dumps, and
KS4 / KS5 / KS Rack / K-Station program and program-pair dumps. The conversion follows Novation's
published program data layout. Envelope, LFO and depth curves are estimated, so patches sound
close to the hardware but not identical. Not reproduced yet: arpeggiator rhythm patterns,
one-shot LFOs, pattern LFO shapes, delay ratio, EQ modulation.

## Building

Jamiro is a project for the MPC Plugin SDK. Put `plugins/Jamiro` inside the SDK's `plugins/`
folder, then:

```
python tools/build.py plugins/Jamiro
python tools/package.py plugins/Jamiro
```

After editing `ui.json`, run `python plugins/Jamiro/tools/make_skin.py` (it regenerates `skins/`
with the background image), then build again. `tools/make_captions.py` (needs Pillow) redraws the
switch-box caption images in `art/captions/` if you change the captions.

Releases are built by GitHub Actions with OpenMPC's release workflow (`.github/workflows/release.yml`).

## Contributing

Ideas, bug reports and pull requests are welcome, especially:
- more accurate X-Station curves (envelope times, LFO rates, filter response);
- arpeggiator rhythm patterns and the missing LFO modes;
- new original programs (please do not commit third-party sound banks);
- CPU optimisation on 32-bit MPCs.

The source is commented: start with `plugins/Jamiro/src/Jamiro.cpp` (signal flow at the top),
`jamiro_dsp.h` (oscillators, LFOs, effects) and `ks_import.h` (the `.syx` reader).
A French README is in `plugins/Jamiro/README.md`.

## Changes

- **1.0.4**: screen design: switch boxes now show their captions in red (MONO / M.AG /
  POLY1 / POLY2, LP / BP / HP, 12 / 24 dB, CHORUS / PHASER), drawn into the button images so the
  MPC always shows them; menu controls (waves, sync, arp mode/rate...) are boxes with readable
  name and value text.
- **1.0.3**: unison no longer crackles (steady level, random start phases, stereo spread);
  transparent look-ahead output limiter (no distortion at high Mix Level / Volume); table waves
  now move from a bright attack to a rounder body (velocity sensitive); new filter with proper
  24 dB Butterworth slope and resonance that self-oscillates at maximum; overdrive keeps a steady
  level; analog-style envelopes with longer, more musical decay/release; finer mod wheel /
  aftertouch vibrato; red, short captions in the switch boxes (LP/BP/HP, 12/24 dB, voice mode).
- **1.0.2**: rebuilt the 9 table waveforms (Organ, Harpsichord, E.Piano, Slap Bass, Rhodes,
  Rhodes Tine, Wurly, Clavinet, Ana Bass) with richer spectra, one band-limited version per
  octave (no more sudden dulling above middle C) and matched loudness; preset levels rebalanced.
  Q-Links now bank by rows: on 4-knob MPCs the knobs, left to right, drive one row of the screen.
- **1.0.1**: neon light-beam skin background.
- **1.0.0**: first release.

## Licence

MIT, see `LICENSE`. The reverb is a classic Schroeder/Moorer network.
