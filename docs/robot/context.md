# Current robot project

This is Team 1466's Java 25 TalonFX swerve starter for SystemCore on WPILib 2027.0.0 Alpha 8. `Robot` owns AdvantageKit logging, replay, and the Commands v3 scheduler. Following the team's 2026 structure, `RobotContainer` selects real, sim, or replay IO and wires the subsystems, driver controls, and autonomous routines together. Register each `FullSubsystem` in the container. Its `periodic()` reads and logs inputs before the scheduler. Commands request goals, the Choreographer enforces drive ownership and neutral interlocks, and `periodicAfterScheduler()` applies the final outputs. Vision fuses logged observations after odometry updates. BLine Beta 3 runs the drive-only starter paths listed in `AutoRoutines.java`. The paths and camera simulator use the 2026 welded practice field. Real drive is disabled and the Luma P1 camera list is empty pending hardware configuration. Game-specific autonomous routines, field selection, and other mechanisms are open.

## Working references

- Keep hardware/prototype revisions and measurement sources under `docs/robot/hardware/` when they exist. The [hardware example](examples/hardware.json) is a documentation example, not an installed mechanism.
- Record shared interface units, ownership, readiness, cancellation, and fault semantics under `docs/robot/interfaces/` when parallel tasks need an agreement.
- Record consequential accepted design decisions and their reasons under `docs/robot/decisions/`. A proposal is not an accepted team decision.
- Keep measured trial results, settings, hardware/code provenance, and artifacts under `docs/robot/experiments/`. An experiment plan contains no observed results yet.

Create records as they become useful; a small code change does not require all of these documents. Existing source and measured evidence take precedence over old plans. Update references when the design changes.

## Team work

Use the team's existing task/review workflow. When available, published issues and branches can describe ownership, dependencies and merge order. They do not expose private chats or another computer's uncommitted work. Coordinate use of physical hardware with the operator and record the revision actually tested.

`frc-toolkit.json` contains this project's verification and coordination settings. Empty checks and unknown hardware remain unconfigured/unknown, not verified. The installation source and version are recorded in `frc-toolkit.lock.json`.
