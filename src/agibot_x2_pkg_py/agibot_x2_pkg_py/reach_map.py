#!/usr/bin/env python3
"""
Helpers for the shelf-slot reachability map: which arms reach a (book, y) slot, without ROS dependencies.
Kept as a table of measurements, not a formula: "grasp only" IK overestimates reach (exit/retreat can miss by 75-100 mm).
Extra measurements go to a JSON file (`X2_REACH_FILE`, default ~/.x2/reach_measured.json).
`python3 -m agibot_x2_pkg_py.reach_map --scan --books 3 --ys -0.20 -0.12`
"""
import json
import os
import subprocess
import sys

TOL = 0.01          # two y closer than 1 cm are the same slot

# (book id, center y) -> arms that reach it, measured with dry-runs and real grasps
MEASURED = {
    (1, 0.047): "R", (1, 0.32): "LR",
    (2, -0.053): "L", (2, 0.22): "LR",
    (4, -0.283): "LR", (4, 0.0): "LR", (4, 0.05): "R", (4, 0.104): "", (4, 0.15): "", (4, 0.2): "L",
    (4, 0.25): "LR", (4, 0.3): "LR",
    (3, -0.161): "R", (3, -0.018): "LR", (3, 0.05): "LR", (3, 0.11): "",
    (3, -0.122): "",        # right arm misses the retreat by 26.6 mm; left cannot grasp it at -0.161
}


def _file():
    """
    Path of the JSON file with extra measurements
    """
    return os.environ.get("X2_REACH_FILE", os.path.join(os.path.expanduser("~"), ".x2", "reach_measured.json"))


def _table():
    """
    Built-in measurements merged with the JSON file
    """
    t = dict(MEASURED)
    try:
        with open(_file(), encoding="utf-8") as f:
            for k, v in json.load(f).items():
                b, y = k.split(":")
                t[(int(b), float(y))] = v
    except (OSError, ValueError):
        pass
    return t


def arms(book, y):
    """
    Arms reaching the slot: "L", "R", "LR", "" (none) or None (not measured)
    """
    for (b, yy), a in _table().items():
        if b == int(book) and abs(yy - float(y)) <= TOL:
            return a
    return None


def move_feasible(book, y_from, y_to):
    """
    True/False if ONE arm reaches both grasp and place (the book stays in that hand),
    None if a measurement is missing (optimistic: try it)
    """
    d = arms(book, y_to)
    if d is None:
        return None
    s = arms(book, y_from)
    cand = set("LR") if s is None else set(s)
    return bool(cand & set(d))


def unreachable_band(margin=0.03, default=((0.075, 0.215),)):
    """
    y band where NO measured book is reachable: the "" points widened by `margin`
    Without measurements returns `default`
    """
    ys = sorted(yy for (b, yy), a in _table().items() if a == "")
    ys = [y for y in ys if y > -0.05]                       # "" at negative y are isolated cases (e.g. It at -0.122)
    if not ys:
        return tuple(default)
    return ((round(ys[0] - margin, 3), round(ys[-1] + margin, 3)),)


def record(book, y, arms_ok):
    """
    Add a measurement to the JSON file (creates the directory)
    """
    path = _file()
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        data = {}
    data[f"{int(book)}:{float(y):.3f}"] = arms_ok
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1, sort_keys=True)


def scan(books, ys, detections="/tmp/x2_detections.json", timeout=900):
    """
    Measure with pick_test_book dry_run (exit code 3 before moving if the pose is unreachable)
    Needs the simulation running. Returns {(book, y): arms}
    """
    out = {}
    for b in books:
        for y in ys:
            ok = ""
            for side, ch in (("left", "L"), ("right", "R")):
                cmd = ["ros2", "run", "agibot_x2_pkg_py", "pick_test_book", "--ros-args", "-p", f"target:={b}",
                       "-p", f"detections_file:={detections}", "-p", "dry_run:=true", "-p", f"dest_y:={y}",
                       "-p", f"arm:={side}", "-p", "head_isbn:=false"]
                rc = subprocess.run(cmd, capture_output=True, timeout=timeout).returncode
                print(f"book {b} y {y:+.3f} {side}: exit {rc}", flush=True)
                if rc == 0:
                    ok += ch
            out[(b, y)] = ok
            record(b, y, ok)
    return out


def main(argv=None):
    """
    CLI: optionally scan, then print the table and the derived unreachable band
    """
    import argparse
    ap = argparse.ArgumentParser(description="shelf slot reachability map")
    ap.add_argument("--scan", action="store_true", help="measure with pick_test_book dry_run (needs the simulation)")
    ap.add_argument("--books", type=int, nargs="*", default=[])
    ap.add_argument("--ys", type=float, nargs="*", default=[])
    a = ap.parse_args(argv)
    if a.scan:
        scan(a.books, a.ys)
    t = _table()
    print(f"{len(t)} measurements; derived unreachable band: {unreachable_band()}")
    for (b, y), v in sorted(t.items()):
        print(f"  book {b}  y {y:+.3f}  arms: {v or 'nessuno'}")


if __name__ == "__main__":
    main(sys.argv[1:])
