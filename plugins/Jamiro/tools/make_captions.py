#!/usr/bin/env python3
"""Draw the switch-box captions (LP/BP/HP, 12/24 dB, voice mode, chorus type) into
copies of the UI kit's segment button images, in red, so the MPC shows them for sure
(text labels laid over buttons are not drawn by the MPC).

Writes art/captions/<key>_<index>_on.png and _off.png. Needs Pillow (pip install pillow)
and the kit images in skins/ (run make_skin.py once first). make_skin.py picks the
results up automatically. Only needed again if the captions or the kit change.
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from make_skin import SHORT_CAPTIONS, caption_key  # noqa: E402

FONT_CANDIDATES = [r"C:\Windows\Fonts\arialbd.ttf", "/Library/Fonts/Arial Bold.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]
RED_ON = (200, 10, 30, 255)     # on the gold "selected" segment
RED_OFF = (255, 70, 80, 255)    # on the dark segments


def font(size):
    for f in FONT_CANDIDATES:
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def main():
    out = os.path.join(PROJECT, "art", "captions")
    os.makedirs(out, exist_ok=True)
    for param, labels in SHORT_CAPTIONS.items():
        n = len(labels)
        for i, text in enumerate(labels):
            for state in ("on", "off"):
                base = Image.open(os.path.join(PROJECT, "skins", "seg%d_%s.png" % (n, state))).convert("RGBA")
                d = ImageDraw.Draw(base)
                size = 22
                while size > 9:
                    f = font(size)
                    l, t, r, b = d.textbbox((0, 0), text, font=f)
                    if r - l <= base.width - 8 and b - t <= base.height - 10:
                        break
                    size -= 1
                l, t, r, b = d.textbbox((0, 0), text, font=f)
                x = (base.width - (r - l)) / 2 - l
                y = (base.height - (b - t)) / 2 - t
                d.text((x, y), text, font=f, fill=RED_ON if state == "on" else RED_OFF)
                base.save(os.path.join(out, "%s_%d_%s.png" % (caption_key(param), i, state)))
    print("captions written to", out)


if __name__ == "__main__":
    main()
