"""Mesh STEP exports of one assembly into AdvantageScope GLB models, grouped by how parts move.

Usage: step2glb.py [--axes -Y,X,Z] [--scale 0.001] OUTDIR BASE.step [MOVED.step ...]

BASE is the assembly at the pose the models are drawn in, usually stowed. Each MOVED export is the
same assembly at another mechanism pose. Parts that move rigidly together between BASE and every
MOVED export form one group. The group that never moves becomes model.glb, and the moving groups
become model_0.glb, model_1.glb, ...: sliding groups by increasing travel, then rotating groups
by increasing angle. A table of the groups, with sample part paths, is printed and saved as
OUTDIR.groups.json so each model can be matched to the component pose the robot code logs. With no MOVED export, everything goes into model.glb.

--axes gives each robot axis (+X forward, +Y left, +Z up) as a signed CAD axis, for example
"-Y,X,Z" when the robot's front faces CAD -Y. --scale converts CAD units to meters; OCP reports
millimeters. The model is shifted so its lowest point is at z = 0. Put the robot's center at the
CAD origin in x and y.

Every mesh has per-vertex normals: AdvantageScope skips a GLB mesh without them, silently.
Needs cadquery-ocp (pip install cadquery-ocp, in a virtual environment).
"""
import argparse
import json
import math
import os
import struct
from array import array

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType, XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence

# Two parts move together when their motions agree within these tolerances (CAD units, and
# rotation matrix entries, about 0.3 degrees).
SAME_TRANSLATION = 1.0
SAME_ROTATION = 0.005
DEFAULT_COLOR = (0.6, 0.6, 0.6)


def read(path):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    r = STEPCAFControl_Reader()
    r.SetColorMode(True)
    r.SetNameMode(True)
    if r.ReadFile(path) != IFSelect_RetDone:
        raise SystemExit(f"Could not read {path}")
    r.Transfer(doc)
    return doc


def name(label):
    a = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return "?"


def color(label, default, color_tool):
    c = Quantity_Color()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        cl = TDF_Label()
        if XCAFDoc_ColorTool.GetColor_s(label, t, cl) and XCAFDoc_ColorTool.GetColor_s(cl, c):
            return (c.Red(), c.Green(), c.Blue())
        try:
            if color_tool.GetColor(XCAFDoc_ShapeTool.GetShape_s(label), t, c):
                return (c.Red(), c.Green(), c.Blue())
        except Exception:
            pass
    return default


def leaves(document):
    """Leaf parts as (ref label, world location, path, color), in document order."""
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(document.Main())
    out = []

    def walk(label, loc, path, rgb):
        comps = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            comp = comps.Value(i)
            cloc = loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(comp))
            ref = TDF_Label()
            XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref)
            p = path + [name(comp)]
            c = color(comp, color(ref, rgb, color_tool), color_tool)
            if XCAFDoc_ShapeTool.IsAssembly_s(ref):
                walk(ref, cloc, p, c)
            else:
                out.append((ref, cloc, p, c))

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [name(roots.Value(i))], DEFAULT_COLOR)
    return out


def motion(base_loc, moved_loc):
    """Rigid motion from the base pose to a moved pose as a 3x4 matrix (rotation | translation)."""
    t = moved_loc.Multiplied(base_loc.Inverted()).Transformation()
    return [[t.Value(r, c) for c in range(1, 5)] for r in range(1, 4)]


IDENTITY = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]


def same_motion(a, b):
    return all(
        abs(a[r][c] - b[r][c]) <= (SAME_TRANSLATION if c == 3 else SAME_ROTATION)
        for r in range(3)
        for c in range(4)
    )


def size(m):
    """Translation length (CAD units) and rotation angle (degrees) of a 3x4 motion."""
    travel = math.sqrt(sum(m[r][3] ** 2 for r in range(3)))
    cos = (m[0][0] + m[1][1] + m[2][2] - 1) / 2
    return travel, math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def parse_axes(text):
    axes = []
    for part in text.split(","):
        part = part.strip().upper()
        sign = -1.0 if part.startswith("-") else 1.0
        letter = part.lstrip("+-")
        if letter not in ("X", "Y", "Z"):
            raise SystemExit(f"Bad --axes entry {part!r}; use signed X, Y, or Z")
        axes.append(("XYZ".index(letter), sign))
    if len(axes) != 3 or len({a for a, _ in axes}) != 3:
        raise SystemExit("--axes needs each of X, Y, and Z once, for example -Y,X,Z")
    return axes


def is_reflection(axes):
    """Whether the CAD-to-robot axis mapping mirrors the model, which reverses triangle winding."""
    m = [[0.0] * 3 for _ in range(3)]
    for row, (axis, sign) in enumerate(axes):
        m[row][axis] = sign
    det = (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )
    return det < 0


def add_shape(shape, rgb, meshes, axes, scale, bounds):
    """Triangulate a placed part into the group's per-color position and index arrays."""
    BRepMesh_IncrementalMesh(shape, 2.5, False, 0.7, True)
    pos, idx = meshes.setdefault(tuple(round(v, 3) for v in rgb), (array("f"), array("I")))
    mirrored = is_reflection(axes)
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(pos) // 3
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                cad = (p.X(), p.Y(), p.Z())
                robot = [sign * cad[axis] * scale for axis, sign in axes]
                pos.extend(robot)
                for k in range(3):
                    bounds[0][k] = min(bounds[0][k], robot[k])
                    bounds[1][k] = max(bounds[1][k], robot[k])
            flip = (face.Orientation() == TopAbs_REVERSED) != mirrored
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if flip:
                    b, c = c, b
                idx.extend((base + a - 1, base + b - 1, base + c - 1))
        exp.Next()


def vertex_normals(pos, idx):
    """Area-weighted per-vertex normals. Each CAD face has its own vertices, so edges stay sharp."""
    acc = [0.0] * len(pos)
    for t in range(0, len(idx), 3):
        a, b, c = idx[t] * 3, idx[t + 1] * 3, idx[t + 2] * 3
        ux, uy, uz = pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]
        vx, vy, vz = pos[c] - pos[a], pos[c + 1] - pos[a + 1], pos[c + 2] - pos[a + 2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        for v in (a, b, c):
            acc[v] += nx
            acc[v + 1] += ny
            acc[v + 2] += nz
    out = array("f", bytes(4 * len(pos)))
    for v in range(0, len(pos), 3):
        n = math.sqrt(acc[v] ** 2 + acc[v + 1] ** 2 + acc[v + 2] ** 2)
        if n > 0:
            out[v], out[v + 1], out[v + 2] = acc[v] / n, acc[v + 1] / n, acc[v + 2] / n
        else:
            out[v + 2] = 1.0
    return out


def write_glb(path, meshes, floor):
    """Write one GLB with a primitive per color. Positions are shifted down by floor."""
    bin_ = bytearray()
    accessors, views, prims, materials = [], [], [], []
    for rgb, (pos, idx) in meshes.items():
        if not idx:
            continue
        pos = array("f", pos)
        for k in range(2, len(pos), 3):
            pos[k] -= floor
        pmin = [min(pos[k::3]) for k in range(3)]
        pmax = [max(pos[k::3]) for k in range(3)]
        off = len(bin_)
        bin_ += pos.tobytes()
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(pos) * 4, "target": 34962})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5126,
                          "count": len(pos) // 3, "type": "VEC3", "min": pmin, "max": pmax})
        pa = len(accessors) - 1
        normals = vertex_normals(pos, idx)
        off = len(bin_)
        bin_ += normals.tobytes()
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(normals) * 4,
                      "target": 34962})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5126,
                          "count": len(normals) // 3, "type": "VEC3"})
        na = len(accessors) - 1
        off = len(bin_)
        bin_ += idx.tobytes()
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(idx) * 4, "target": 34963})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5125, "count": len(idx),
                          "type": "SCALAR"})
        materials.append({"pbrMetallicRoughness": {"baseColorFactor": [*rgb, 1.0],
                                                   "metallicFactor": 0.1, "roughnessFactor": 0.7},
                          "doubleSided": True})
        prims.append({"attributes": {"POSITION": pa, "NORMAL": na},
                      "indices": len(accessors) - 1, "material": len(materials) - 1})
    gltf = {"asset": {"version": "2.0", "generator": "step2glb"}, "scene": 0,
            "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": prims}], "materials": materials, "accessors": accessors,
            "bufferViews": views, "buffers": [{"byteLength": len(bin_)}]}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    bin_ += b"\0" * (-len(bin_) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        f.write(struct.pack("<II", len(bin_), 0x004E4942) + bin_)
    print(path, round(total / 1e6, 2), "MB")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("outdir")
    parser.add_argument("base", metavar="BASE.step")
    parser.add_argument("moved", metavar="MOVED.step", nargs="*")
    parser.add_argument("--axes", default="X,Y,Z", help="robot X,Y,Z as signed CAD axes")
    parser.add_argument("--scale", type=float, default=0.001, help="meters per CAD unit")
    args = parser.parse_args()
    axes = parse_axes(args.axes)

    # Keep the documents referenced: their labels crash OCP once a document is collected.
    docs = [read(args.base)] + [read(path) for path in args.moved]
    base_leaves = leaves(docs[0])
    moved_leaves = [leaves(doc) for doc in docs[1:]]
    for path, other in zip(args.moved, moved_leaves):
        if len(other) != len(base_leaves) or any(a[2] != b[2] for a, b in zip(base_leaves, other)):
            raise SystemExit(f"{path} does not have the same assembly structure as {args.base}")

    # Group parts whose motion to every moved export agrees.
    groups = []
    bounds = [[math.inf] * 3, [-math.inf] * 3]
    for i, (ref, loc, path, rgb) in enumerate(base_leaves):
        motions = [motion(loc, other[i][1]) for other in moved_leaves]
        group = next((g for g in groups
                      if all(same_motion(a, b) for a, b in zip(g["motions"], motions))), None)
        if group is None:
            group = {"motions": motions, "paths": [], "meshes": {}}
            groups.append(group)
        group["paths"].append("/".join(path))
        shape = XCAFDoc_ShapeTool.GetShape_s(ref).Moved(loc)
        add_shape(shape, rgb, group["meshes"], axes, args.scale, bounds)

    def is_static(g):
        return all(same_motion(m, IDENTITY) for m in g["motions"])

    def order(g):
        # Sliding groups by travel, then rotating groups. A rotating part's travel depends on where
        # its CAD origin is, so it does not say how far along a mechanism chain the part sits.
        travel = max((size(m)[0] for m in g["motions"]), default=0.0)
        angle = max((size(m)[1] for m in g["motions"]), default=0.0)
        return (angle > 0.5, angle if angle > 0.5 else travel)

    static = [g for g in groups if is_static(g)]
    moving = sorted((g for g in groups if not is_static(g)), key=order)
    files = [("model.glb", g) for g in static] + [(f"model_{n}.glb", g)
                                                   for n, g in enumerate(moving)]
    if not static:
        print("warning: no part stays still, so there is no model.glb for the robot base")

    floor = bounds[0][2]
    os.makedirs(args.outdir, exist_ok=True)
    report = []
    for fname, g in files:
        write_glb(os.path.join(args.outdir, fname), g["meshes"], floor)
        motions = [{"export": os.path.basename(p), "travel": round(size(m)[0], 2),
                    "rotation_deg": round(size(m)[1], 2)}
                   for p, m in zip(args.moved, g["motions"])]
        report.append({"model": fname, "parts": len(g["paths"]), "motion": motions,
                       "sample_paths": g["paths"][:5]})
        moves = ", ".join(f"{m['travel']} / {m['rotation_deg']} deg" for m in motions) or "-"
        print(f"  {len(g['paths'])} parts; travel (CAD units) / rotation per export: {moves}")
        print(f"  e.g. {g['paths'][0]}")
    if len(moving) > 3:
        print("note: many moving groups usually means a flexible part (belt, chain, cable) or a"
              " pose that moves parts that should be rigid; check the sample paths")
    print("robot-frame bounds before the floor shift (m)", bounds)

    with open(os.path.normpath(args.outdir) + ".groups.json", "w") as f:
        json.dump({"floor_shift_m": -floor, "bounds_m": bounds, "models": report}, f, indent=2)
    config = os.path.join(args.outdir, "config.json")
    if not os.path.exists(config):
        robot = os.path.basename(os.path.normpath(args.outdir)).removeprefix("Robot_")
        with open(config, "w") as f:
            json.dump({"name": robot, "isFTC": False, "disableSimplification": False,
                       "rotations": [], "position": [0, 0, 0], "cameras": [],
                       "components": [{"zeroedRotations": [], "zeroedPosition": [0, 0, 0]}
                                      for _ in moving]}, f, indent=2)
        print("wrote", config)


if __name__ == "__main__":
    main()
