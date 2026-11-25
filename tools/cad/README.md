# CAD helpers

These scripts turn STEP exports of the robot assembly into mechanism geometry and AdvantageScope models. Export the same top-level assembly once per mechanism pose, for example stowed, scoring, and intaking. The exports must have identical structure, so change only mates or mechanism positions between them.

| Script | Needs | Does |
| --- | --- | --- |
| `steptree.py OUT.json POSE.step [POSE.step ...]` | Python 3 only | Reads each export's assembly tree and part placements into one JSON file, keyed by file name. |
| `pivot.py OUT.json FROM TO` | Python 3 only | Lists every part with its travel and rotation from pose `FROM` to pose `TO`. |
| `pivot.py OUT.json FROM TO PART [--relative-to CARRIER]` | Python 3 only | Reports one part's translation, rotation angle and axis, and pivot point. `--relative-to` removes a carrier's motion first, for example an arm on an elevator carriage. |
| `step2glb.py [--axes -Y,X,Z] OUTDIR BASE.step [MOVED.step ...]` | [`cadquery-ocp`](https://pypi.org/project/cadquery-ocp/) | Meshes `BASE` into AdvantageScope GLB models in robot coordinates, one per group of parts that move together. |

`steptree.py` and `pivot.py` report lengths in the STEP file's unit and use CAD axes. Convert both to robot units and axes before putting values in code, and record which export each value came from.

## AdvantageScope models

Install OCP in a virtual environment (`python -m venv .venv`, then `pip install cadquery-ocp` with that environment's pip). Then run:

```
python tools/cad/step2glb.py --axes -Y,X,Z advantagescope/Robot_Name stowed.step scoring.step intaking.step
```

- **Grouping.** Parts that move rigidly together between `BASE` and every `MOVED` export form one group. The group that never moves becomes `model.glb`. Moving groups become `model_0.glb`, `model_1.glb`, and so on: sliding groups by increasing travel, then rotating groups by increasing angle. The script prints each group with its motion and a sample part path and saves the same table as `OUTDIR.groups.json`. Check it: each model index must match the order of the component poses the robot code logs. Add an export when two mechanisms happen to move together in every pose you have exported. A flexible part such as a belt or chain may form its own small group.
- **Axes.** `--axes` lists the CAD axis for the robot's +X (forward), +Y (left), and +Z (up). `-Y,X,Z` means the robot's front faces CAD -Y. Put the robot's center at the CAD origin in x and y. The script shifts the model so its lowest point is at z = 0. `--scale` converts OCP's millimeters to meters.
- **Normals.** Every mesh includes per-vertex normals. AdvantageScope silently skips a GLB mesh without normals, so keep them when changing the writer.
- **Config.** If `OUTDIR/config.json` does not exist, the script writes one with a zeroed component per moving group. Models are drawn at the `BASE` pose, so each logged component pose is the motion away from that pose.

Point AdvantageScope's custom assets folder at the folder that contains `Robot_Name` (see AdvantageScope's custom assets documentation), select the robot in a 3D field view, and add the logged component poses array to it.
