"""Measure how parts move between two exported poses of the same assembly. Pure Python.

Usage:
  pivot.py TREE.json FROM TO
      List every leaf part with its translation and rotation from pose FROM to pose TO. Use this
      to find parts that move together and to pick names for the next form.
  pivot.py TREE.json FROM TO PART [--relative-to CARRIER]
      Report one part's motion: translation, rotation angle and axis, and the pivot point (the
      point on the rotation axis nearest the origin) in pose FROM's coordinates. With CARRIER, the
      carrier's motion is removed first, giving, for example, an arm's rotation about its pivot
      on a moving elevator carriage.

TREE.json comes from steptree.py. FROM and TO are export names in it. PART and CARRIER are
case-sensitive substrings of a leaf path; the first matching leaf is used. Lengths are in the STEP
file's unit and axes are CAD axes, so convert them to robot units and axes before using them in code.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from steptree import inv, mul  # noqa: E402


def motion(tree, frm, to, i):
    """Rigid transform taking leaf i from pose FROM to pose TO, in world coordinates."""
    return mul(tree[to][i]["M"], inv(tree[frm][i]["M"]))


def rotation(m):
    """Angle in degrees and unit axis of a transform's rotation."""
    trace = m[0][0] + m[1][1] + m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1) / 2)))
    axis = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]]
    n = math.sqrt(sum(a * a for a in axis))
    if n < 1e-9:
        if angle < 1e-6:
            return 0.0, [0.0, 0.0, 0.0]
        # Half turn: the axis is the column of R + I with the largest norm.
        cols = [[m[r][c] + (1 if r == c else 0) for r in range(3)] for c in range(3)]
        col = max(cols, key=lambda v: sum(x * x for x in v))
        n = math.sqrt(sum(x * x for x in col))
        return math.degrees(angle), [x / n for x in col]
    return math.degrees(angle), [a / n for a in axis]


def solve3(a, b):
    det = (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )
    out = []
    for k in range(3):
        m = [row[:] for row in a]
        for r in range(3):
            m[r][k] = b[r]
        out.append(
            (
                m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
            )
            / det
        )
    return out


def pivot_point(m, axis):
    """Least-squares solution of (I - R) p = t with p perpendicular to the axis."""
    rows = [[(1 if r == c else 0) - m[r][c] for c in range(3)] for r in range(3)] + [axis]
    rhs = [m[0][3], m[1][3], m[2][3], 0.0]
    ata = [[sum(rows[k][i] * rows[k][j] for k in range(4)) for j in range(3)] for i in range(3)]
    atb = [sum(rows[k][i] * rhs[k] for k in range(4)) for i in range(3)]
    return solve3(ata, atb)


def fmt(v):
    return "(" + ", ".join(f"{x:.4f}" for x in v) + ")"


def find(tree, frm, text):
    hits = [i for i, e in enumerate(tree[frm]) if e["leaf"] and text in e["path"]]
    if not hits:
        sys.exit(f"No leaf path contains {text!r}. Run without PART to list them.")
    return hits[0], len(hits)


def main(argv):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("tree")
    parser.add_argument("frm", metavar="FROM")
    parser.add_argument("to", metavar="TO")
    parser.add_argument("part", nargs="?")
    parser.add_argument("--relative-to", dest="carrier")
    opts = parser.parse_args(argv)
    args = [opts.tree, opts.frm, opts.to] + ([opts.part] if opts.part else [])
    carrier_text = opts.carrier
    with open(args[0]) as f:
        tree = json.load(f)
    frm, to = args[1], args[2]
    for name in (frm, to):
        if name not in tree:
            sys.exit(f"No export named {name!r}. Exports: {', '.join(tree)}")
    if len(tree[frm]) != len(tree[to]):
        sys.exit("The two exports have different structures, so their parts do not line up.")

    if len(args) == 3:
        print("translation  rotation(deg)  path")
        for i, e in enumerate(tree[frm]):
            if e["leaf"]:
                m = motion(tree, frm, to, i)
                travel = math.sqrt(sum(m[r][3] ** 2 for r in range(3)))
                print(f"{travel:11.4f}  {rotation(m)[0]:13.2f}  {e['path']}")
        return

    part, count = find(tree, frm, args[3])
    print(f"part: {tree[frm][part]['path']} ({count} matching leaves)")
    m = motion(tree, frm, to, part)
    if carrier_text is not None:
        carrier, count = find(tree, frm, carrier_text)
        c = motion(tree, frm, to, carrier)
        print(f"carrier: {tree[frm][carrier]['path']} ({count} matching leaves)")
        print(f"carrier translation {fmt([c[r][3] for r in range(3)])}, "
              f"magnitude {math.sqrt(sum(c[r][3] ** 2 for r in range(3))):.4f}, "
              f"rotation {rotation(c)[0]:.2f} deg")
        m = mul(inv(c), m)
    angle, axis = rotation(m)
    print(f"translation {fmt([m[r][3] for r in range(3)])}")
    if angle < 0.01:
        print("rotation none")
        return
    print(f"rotation {angle:.2f} deg about axis {fmt(axis)} (right-hand rule)")
    print(f"pivot point in pose {frm!r} coordinates {fmt(pivot_point(m, axis))}")


if __name__ == "__main__":
    main(sys.argv[1:])
