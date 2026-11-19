#!/usr/bin/env python3
"""Build Jamiro's touch-screen skin into plugins/Jamiro/skins/.

1. Runs tools/gen_ui.py on ui.json (layout, kit colours, label colours).
2. Puts a 1280x628 background image behind every page.

tools/build.py then copies skins/ verbatim. Re-run this script after editing
ui.json or the parameters (it needs build/params.json from a previous build):

    python plugins/Jamiro/tools/make_skin.py                       # art/jamiro_bg_public.png
    python plugins/Jamiro/tools/make_skin.py --bg art/mine.png     # your own 1280x628 picture
    python plugins/Jamiro/tools/make_skin.py --plain               # plain violet, no image
    python tools/build.py plugins/Jamiro
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
SDK = os.path.dirname(os.path.dirname(PROJECT))
COLOUR = "ff5848ad"
ROW_Y = (45, 196, 346, 497)   # top of each control row in the generated 4 x 4 grid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bg", default="art/jamiro_bg_public.png", help="background PNG (path relative to plugins/Jamiro)")
    ap.add_argument("--plain", action="store_true", help="no background image")
    ap.add_argument("--params", default=os.path.join(PROJECT, "build", "params.json"))
    args = ap.parse_args()

    skins = os.path.join(PROJECT, "skins")
    if os.path.isdir(skins):
        shutil.rmtree(skins)
    subprocess.check_call([sys.executable, os.path.join(SDK, "tools", "gen_ui.py"),
                           "--params", args.params,
                           "--ui", os.path.join(PROJECT, "ui.json"),
                           "--out", skins,
                           "--kit-cache", os.path.join(PROJECT, "build")])
    image = ""
    if not args.plain:
        image = "background.png"
        shutil.copy(os.path.join(PROJECT, args.bg), os.path.join(skins, image))
    tui_path = os.path.join(skins, "TUI.json")
    with open(tui_path, encoding="utf-8") as f:
        tui = json.load(f)
    for tab in tui["pageData"]["tabs"]:
        bd = tab["componentDefinition"]["backgroundData"]
        for state in ("focussed", "unfocussed"):
            bd[state] = {"version": 1, "colour": COLOUR, "image": image}
        # Q-Link groups are ROWS: the focus rectangle wraps one row of four controls.
        tab["qlinkBoundsData"] = ["13 %d 1254 121" % (y - 7) for y in ROW_Y]
    with open(tui_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(tui, f, indent=2)

    # Bank the knobs by rows, top row first: on a 4-knob MPC the knobs, left to right,
    # drive one on-screen row (e.g. Cutoff, Resonance, Overdrive, Key Track).
    q_path = os.path.join(skins, "Q-Links.json")
    with open(q_path, encoding="utf-8") as f:
        ql = json.load(f)
    for entry in ql["Screen Mode Q-Links"]["map"]:
        entry["Bank Direction"] = "RowTopDown"
    with open(q_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(ql, f, indent=2)
    print("skin written to", skins, "background:", image or "none")


if __name__ == "__main__":
    main()
