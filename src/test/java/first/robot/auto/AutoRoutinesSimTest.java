package first.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.generated.TunerConstants;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
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
  void redAllianceFlipsTheJavaPathOnce() {
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    try {
      autos.updateInputs();
      String status = runToEnd(AutoRoutine.singlePath("Red", 5, AutoPaths::drive1Meter));
      assertEquals("Completed", status);
      var field = first.robot.subsystems.vision.VisionConstants.aprilTagLayout;
      // Blue (3, 4, 0 deg) rotated about the field center.
      assertEquals(field.getFieldLength() - 3.0, drive.getPose().getX(), 0.08);
      assertEquals(field.getFieldWidth() - 4.0, drive.getPose().getY(), 0.08);
      assertEquals(180, Math.abs(drive.getPose().getRotation().getDegrees()), 3);
    } finally {
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
