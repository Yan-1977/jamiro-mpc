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
# Short captions that fit inside the segment buttons (the full names stay in the value text).
# Pages shown as a second page of another function key: {page: parent page}.
SUB_PAGES = {"LFO MENU": "LFO/ARP"}

SHORT_CAPTIONS = {
    "Filter Type": ["LP", "BP", "HP"],
    "Slope": ["12 dB", "24 dB"],
    "Voice Mode": ["MONO", "M.AG", "POLY1", "POLY2"],
    "Chorus Type": ["CHORUS", "PHASER"],
    "LFO1 KeySync": ["OFF", "ON"],
    "LFO1 Unipol": ["OFF", "ON"],
    "LFO1 OneShot": ["OFF", "ON"],
    "LFO1 Common": ["OFF", "ON"],
    "LFO2 KeySync": ["OFF", "ON"],
    "LFO2 Unipol": ["OFF", "ON"],
    "LFO2 OneShot": ["OFF", "ON"],
    "LFO2 Common": ["OFF", "ON"],
}


def caption_key(param):
    return param.lower().replace(" ", "_")


NAME_COLOUR = "ffc9f3ff"    # same as the knob names
VALUE_COLOUR = "ffffe9a6"   # same as the knob values


def _entry(name, typ, data, bounds):
    return {"version": 2,
            "componentData": {"version": 1, "name": name, "type": typ, "data": data},
            "handle remapping": {"version": 1, "map": []},
            "bounds": {"version": 1, "acceptsHWFocus": "No", "showWhenDataModelInvalid": "Show",
                       "whenVisible": "Always", "boundsType": "Absolute", "bounds": bounds}}


def _label_entry(name, kind, colour, height, bounds):
    return _entry(name, "Label", {"version": 1,
                                  "textStyle": {"version": 1,
                                                "font": {"version": 1, "name": "Titillium Web", "style": "Regular", "height": height},
                                                "colour": colour, "justification": "left", "case": "Original"},
                                  "type": kind, "handleName": "Data"}, bounds)


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
    # Menu controls (LFO wave, sync, arp mode/rate ...): the MPC's generic bar draws its
    # text in a fixed dark colour that disappears on the violet background. Replace it with
    # a box (dark panel whose fill shows the position) and our own name / value labels.
    for d in tui["pageData"]["componentDefinitions"]["localComponentDefinitions"]:
        if d["key"] not in ("sliderWithFocus", "sliderWithFocusMenu"):
            continue
        kids = d["value"]["componentsData"]
        old = [c for c in kids if c["componentData"]["type"] == "Slider"]
        if not old:
            continue
        i = kids.index(old[0])
        kids[i:i + 1] = [
            _entry("box", "Slider", {"version": 2, "sliderType": "ColourBar", "direction": "Horizontal",
                                     "backgroundColour": "e0221a52", "outlineColour": "ffb9b0ff",
                                     "valueColour": "e0453a9c", "isBipolar": False, "handleName": "Data"},
                   "10 6 258 95"),
            _label_entry("name", "Name", NAME_COLOUR, 24.0, "22 10 240 32"),
            _label_entry("value", "Value", VALUE_COLOUR, 31.0, "22 44 240 44"),
        ]

    # Switch boxes: draw the captions INTO the button images (art/captions, made by
    # make_captions.py). Text labels laid over the buttons are not drawn by the MPC.
    defs = {d["key"]: d for d in tui["pageData"]["componentDefinitions"]["localComponentDefinitions"]}
    cap_dir = os.path.join(PROJECT, "art", "captions")
    captioned = set()
    for param, labels in SHORT_CAPTIONS.items():
        files = [("%s_%d_on.png" % (caption_key(param), i), "%s_%d_off.png" % (caption_key(param), i)) for i in range(len(labels))]
        base_key = "radio%d" % len(labels)
        if base_key not in defs or not all(os.path.exists(os.path.join(cap_dir, f)) for pair in files for f in pair):
            continue
        clone = json.loads(json.dumps(defs[base_key]))
        clone["key"] = "radio_" + caption_key(param)
        for c in clone["value"]["componentsData"]:
            data = c["componentData"]["data"]
            if c["componentData"]["type"] == "Button":
                on, off = files[data["buttonId"]]
                data["onImage"], data["offImage"] = on, off
        for pair in files:
            for f in pair:
                shutil.copy(os.path.join(cap_dir, f), os.path.join(skins, f))
        tui["pageData"]["componentDefinitions"]["localComponentDefinitions"].append(clone)
        captioned.add(param)

    for tab in tui["pageData"]["tabs"]:
        kids = tab["componentDefinition"]["componentsData"]
        for c in kids:
            cd = c["componentData"]
            # (OFF / ON settings come out of gen_ui as pill toggles: they become captioned boxes too)
            if cd["name"] in captioned and (cd["type"].startswith("radio") or cd["type"] == "toggle"):
                cd["type"] = "radio_" + caption_key(cd["name"])
        # drop the old overlay captions of switches whose images now carry the text
        kids[:] = [c for c in kids if not (c["componentData"]["type"] == "Label" and " seg " in c["componentData"]["name"]
                                          and c["componentData"]["name"].rpartition(" seg ")[0] in captioned)]
        bd = tab["componentDefinition"]["backgroundData"]
        for state in ("focussed", "unfocussed"):
            bd[state] = {"version": 1, "colour": COLOUR, "image": image}
        # Q-Link groups are ROWS: the focus rectangle wraps one row of four controls.
        tab["qlinkBoundsData"] = ["13 %d 1254 121" % (y - 7) for y in ROW_Y]
    # Sub-pages: a page listed in SUB_PAGES sits on its parent's function key and is reached
    # by pressing that key again (the MPC has six function keys).
    tabs = tui["pageData"]["tabs"]
    moved = {}   # old Q-Link "Tab" number -> (new Tab, new SubTab)
    for child, parent in SUB_PAGES.items():
        c = next((t for t in tabs if t["tabName"] == child), None)
        par = next((t for t in tabs if t["tabName"] == parent), None)
        if c is None or par is None:
            continue
        subs = [t for t in tabs if t["fnKeyIndex"] == par["fnKeyIndex"] and t is not c]
        moved[c["fnKeyIndex"] + 1] = (par["fnKeyIndex"] + 1, len(subs) + 1)
        c["fnKeyIndex"], c["fnKeySubIndex"] = par["fnKeyIndex"], len(subs)
        tabs.remove(c)
        tabs.insert(tabs.index(subs[-1]) + 1, c)
    with open(tui_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(tui, f, indent=2)

    # Bank the knobs by rows, top row first: on a 4-knob MPC the knobs, left to right,
    # drive one on-screen row (e.g. Cutoff, Resonance, Overdrive, Key Track).
    q_path = os.path.join(skins, "Q-Links.json")
    with open(q_path, encoding="utf-8") as f:
        ql = json.load(f)
    for entry in ql["Screen Mode Q-Links"]["map"]:
        entry["Bank Direction"] = "RowTopDown"
        if entry.get("SubTab") == 1 and entry.get("Tab") in moved:
            entry["Tab"], entry["SubTab"] = moved[entry["Tab"]]
    with open(q_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(ql, f, indent=2)
    print("skin written to", skins, "background:", image or "none")


if __name__ == "__main__":
    main()
