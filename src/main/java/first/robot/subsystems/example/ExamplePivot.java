package first.robot.subsystems.example;

import static first.robot.subsystems.example.ExamplePivotConstants.*;

import first.robot.util.FullSubsystem;
import first.robot.util.GoalTimeout;
import first.robot.util.LoggedTunableNumber;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Holds the pivot angle goal and applies it after commands and interlocks run. */
public class ExamplePivot extends FullSubsystem {
  private final ExamplePivotIO io;
  private final ExamplePivotIOInputsAutoLogged inputs = new ExamplePivotIOInputsAutoLogged();
  private final Alert[] disconnectedAlerts = new Alert[MOTOR_NAMES.length];
  private final Alert[] hotAlerts = new Alert[MOTOR_NAMES.length];
  private final GoalTimeout goalTimeout =
      new GoalTimeout("ExamplePivotGoalTimeout", "Pivot has not reached its goal. Holding goal.");

  private final LoggedTunableNumber kP = new LoggedTunableNumber("ExamplePivot/kP", 30.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("ExamplePivot/kD", 1.0);
  private final LoggedTunableNumber cruise =
      new LoggedTunableNumber("ExamplePivot/CruiseRadPerSec", 6.0);
  private final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("ExamplePivot/AccelerationRadPerSecSq", 20.0);
  private final LoggedTunableNumber timeoutSeconds =
      new LoggedTunableNumber("ExamplePivot/GoalTimeoutSeconds", 2.0);
  private double[] appliedGains = new double[0];

  private double goalRad = STOW_ANGLE_RAD;
  private boolean neutralRequested = false;

  public ExamplePivot(ExamplePivotIO io) {
    this.io = io;
    for (int i = 0; i < MOTOR_NAMES.length; i++) {
      String name = MOTOR_NAMES[i];
      disconnectedAlerts[i] =
          new Alert(
              "ExamplePivot" + name,
              "Pivot " + name.toLowerCase() + " motor disconnected.",
              Level.HIGH);
      hotAlerts[i] =
          new Alert(
              "ExamplePivot" + name + "Hot",
              "Pivot " + name.toLowerCase() + " motor above " + (int) HOT_MOTOR_CELSIUS + " C.",
              Level.MEDIUM);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ExamplePivot", inputs);
    for (int i = 0; i < MOTOR_NAMES.length; i++) {
      // A motor missing from the inputs, such as with a no-op IO, counts as disconnected.
      disconnectedAlerts[i].set(i >= inputs.connected.length || !inputs.connected[i]);
      hotAlerts[i].set(i < inputs.tempCelsius.length && inputs.tempCelsius[i] > HOT_MOTOR_CELSIUS);
    }
  }

  /** Set the angle goal, clamped to the soft limits. The goal survives disable. */
  public void setGoal(double positionRad) {
    goalRad = Math.clamp(positionRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
  }

  /** Stop driving the motors for this loop. */
  public void neutral() {
    neutralRequested = true;
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getGoalRad() {
    return goalRad;
  }

  /**
   * Whether the pivot is within tolerance of the goal it is currently holding. A coordinator that
   * just requested a new goal should use {@link #isNear} with that goal instead; see the package
   * documentation.
   */
  public boolean atGoal() {
    return isNear(goalRad);
  }

  /**
   * Whether the measured angle is within tolerance of an angle, with the leader connected. The
   * leader's encoder is the position source.
   */
  public boolean isNear(double positionRad) {
    return inputs.connected.length > 0
        && inputs.connected[0]
        && Math.abs(inputs.positionRad - positionRad) < AT_GOAL_TOLERANCE_RAD;
  }

  @Override
  public void periodicAfterScheduler() {
    double[] gains = {kP.get(), kD.get(), cruise.get(), acceleration.get()};
    if (!Arrays.equals(gains, appliedGains)) {
      io.configure(gains[0], gains[1], gains[2], gains[3]);
      appliedGains = gains;
    }
    if (neutralRequested) {
      io.neutral();
    } else {
      io.runPosition(goalRad);
    }
    boolean atGoal = atGoal();
    boolean timedOut = goalTimeout.update(!neutralRequested, goalRad, atGoal, timeoutSeconds.get());

    Logger.recordOutput("ExamplePivot/GoalDegrees", Math.toDegrees(goalRad));
    Logger.recordOutput("ExamplePivot/PositionDegrees", Math.toDegrees(inputs.positionRad));
    Logger.recordOutput("ExamplePivot/Neutral", neutralRequested);
    Logger.recordOutput("ExamplePivot/AtGoal", atGoal);
    Logger.recordOutput(
        "Diagnostics/ExamplePivot/GoalErrorDegrees", Math.toDegrees(goalRad - inputs.positionRad));
    Logger.recordOutput(
        "Diagnostics/ExamplePivot/TrackingErrorDegrees",
        Math.toDegrees(inputs.setpointRad - inputs.positionRad));
    Logger.recordOutput("Diagnostics/ExamplePivot/SecondsOnGoal", goalTimeout.elapsedSeconds());
    Logger.recordOutput("Diagnostics/ExamplePivot/GoalTimedOut", timedOut);
    Logger.recordOutput(
        "Diagnostics/ExamplePivot/TotalStatorCurrentAmps",
        Arrays.stream(inputs.statorCurrentAmps).sum());
    neutralRequested = false;
  }
}
