package first.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import bline.field.FlippingUtil;
import bline.path.Path;
import bline.path.Path.Waypoint;
import first.robot.generated.TunerConstants;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Runs autonomous routines against the simulated swerve and BLine. This checks sequencing, pose
 * reset, completion, and fault reporting with the ideal module simulation. It says nothing about
 * real carpet, wheel slip, or tuned gains.
 */
class AutoRoutinesSimTest {
  private static Drive drive;
  private static AutoRoutines autos;
  private static final Scheduler scheduler = Scheduler.getDefault();

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize());
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
    autos = new AutoRoutines(drive);
    autos.updateInputs();
  }

  private static String runToEnd(AutoRoutine routine) {
    Command command = autos.run(routine);
    assertTrue(scheduler.schedule(command).successful(), routine.name());
    for (int loop = 0; loop < 50 * 30 && scheduler.isScheduledOrRunning(command); loop++) {
      drive.periodic();
      // Applies the BLine gains. "false" keeps the chooser from starting its own routine.
      autos.periodic(false);
      scheduler.run();
      drive.periodicAfterScheduler();
      SimHooks.stepTiming(0.02);
    }
    assertTrue(!scheduler.isScheduledOrRunning(command), routine.name() + " never ended");
    return autos.lastStatus();
  }

  @Test
  void everyRoutineCompletesInSimulation() {
    for (var routine : autos.routines()) {
      String status = runToEnd(routine);
      Pose2d pose = drive.getPose();
      System.out.printf(
          "%s -> %s, final pose (%.3f, %.3f, %.1f deg)%n",
          routine.name(), status, pose.getX(), pose.getY(), pose.getRotation().getDegrees());
      assertEquals("Completed", status, routine.name());
    }
  }

  @Test
  void outAndBackEndsAtItsStart() {
    String status =
        runToEnd(
            new AutoRoutine(
                "Out and Back Check",
                12,
                auto -> {
                  if (!auto.follow(AutoPaths.drive1Meter())) return;
                  auto.follow(AutoPaths.driveBack1Meter());
                }));
    assertEquals("Completed", status);
    assertEquals(2.0, drive.getPose().getX(), 0.08);
    assertEquals(4.0, drive.getPose().getY(), 0.08);
  }

  @Test
  void redAllianceTransformsTheJavaPathOnce() {
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    try {
      autos.updateInputs();
      String status = runToEnd(AutoRoutine.singlePath("Red", 5, AutoPaths::drive1Meter));
      assertEquals("Completed", status);
      // Blue end pose (3, 4, 0 deg), transformed with the field's configured symmetry.
      Pose2d expected = FlippingUtil.flipFieldPose(new Pose2d(3, 4, Rotation2d.ZERO));
      Pose2d actual = drive.getPose();
      assertEquals(expected.getX(), actual.getX(), 0.08);
      assertEquals(expected.getY(), actual.getY(), 0.08);
      assertEquals(
          0, actual.getRotation().minus(expected.getRotation()).getDegrees(), 3, "heading");
    } finally {
      DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
      DriverStationSim.notifyNewData();
      autos.updateInputs();
    }
  }

  /**
   * A path far from both center lines with a diagonal heading, where rotational and mirrored flips
   * disagree in y and heading. A path along the field's center line cannot tell them apart.
   */
  @ParameterizedTest
  @EnumSource(FlippingUtil.FieldSymmetry.class)
  void redAllianceUsesTheConfiguredFieldSymmetry(FlippingUtil.FieldSymmetry symmetry) {
    var original = FlippingUtil.symmetryType;
    FlippingUtil.symmetryType = symmetry;
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    try {
      autos.updateInputs();
      Rotation2d heading = Rotation2d.fromDegrees(30);
      String status =
          runToEnd(
              AutoRoutine.singlePath(
                  "Red Off Center",
                  8,
                  () ->
                      new Path(new Waypoint(2.0, 1.5, heading), new Waypoint(3.5, 2.0, heading))));
      assertEquals("Completed", status, symmetry.name());

      // Written out instead of calling FlippingUtil, so the test does not share the code under
      // test.
      double length = FlippingUtil.fieldSizeX;
      double width = FlippingUtil.fieldSizeY;
      Pose2d expected =
          switch (symmetry) {
            case kRotational ->
                new Pose2d(length - 3.5, width - 2.0, heading.plus(Rotation2d.k180deg));
            case kMirrored -> new Pose2d(length - 3.5, 2.0, Rotation2d.k180deg.minus(heading));
          };
      Pose2d actual = drive.getPose();
      assertEquals(expected.getX(), actual.getX(), 0.08, symmetry + " x");
      assertEquals(expected.getY(), actual.getY(), 0.08, symmetry + " y");
      assertEquals(
          0,
          actual.getRotation().minus(expected.getRotation()).getDegrees(),
          3,
          symmetry + " heading");
    } finally {
      FlippingUtil.symmetryType = original;
      DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
      DriverStationSim.notifyNewData();
      autos.updateInputs();
    }
  }

  @Test
  void routineTimeoutIsReportedAsFailure() {
    String status = runToEnd(AutoRoutine.singlePath("Too Short", 0.5, AutoPaths::drive1Meter));
    assertTrue(status.startsWith("Timed out"), status);
  }

  @Test
  void segmentTimeoutStopsLaterSteps() {
    boolean[] reachedLaterStep = {false};
    String status =
        runToEnd(
            new AutoRoutine(
                "Segment Timeout",
                10,
                auto -> {
                  if (!auto.follow(AutoPaths.drive1Meter(), 0.3)) return;
                  reachedLaterStep[0] = true;
                }));
    assertTrue(status.startsWith("Path did not reach its endpoint"), status);
    assertTrue(!reachedLaterStep[0]);
  }

  @Test
  void missingDrawnPathIsReported() {
    String status =
        runToEnd(new AutoRoutine("Missing Drawn", 5, auto -> auto.followDrawn("does-not-exist")));
    assertTrue(status.startsWith("Could not load drawn path does-not-exist"), status);
  }
}
