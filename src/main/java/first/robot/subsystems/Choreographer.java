package first.robot.subsystems;

import first.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Resolves robot-level goals into subsystem ownership and stop interlocks. */
public final class Choreographer {
  private enum Goal {
    IDLE,
    DRIVER_CONTROL,
    AUTONOMOUS,
    CHARACTERIZATION
  }

  public enum State {
    IDLE,
    WAITING_FOR_CONTROLLER,
    DRIVER_CONTROL,
    AUTONOMOUS,
    CHARACTERIZATION
  }

  private final Drive drive;
  private final Alert controllerAlert =
      new Alert("DriverController", "Driver controller on port 0 is disconnected.", Level.MEDIUM);
  private State state = State.IDLE;

  public Choreographer(Drive drive) {
    this.drive = drive;
  }

  /** Resolve ownership from inputs that callers have already recorded for replay. */
  public State periodic(
      boolean autoEnabled,
      boolean teleopEnabled,
      boolean testEnabled,
      boolean controllerConnected) {
    Goal goal =
        autoEnabled
            ? Goal.AUTONOMOUS
            : teleopEnabled ? Goal.DRIVER_CONTROL : testEnabled ? Goal.CHARACTERIZATION : Goal.IDLE;
    state =
        switch (goal) {
          case IDLE -> State.IDLE;
          case AUTONOMOUS -> State.AUTONOMOUS;
          case DRIVER_CONTROL ->
              controllerConnected ? State.DRIVER_CONTROL : State.WAITING_FOR_CONTROLLER;
          case CHARACTERIZATION ->
              controllerConnected ? State.CHARACTERIZATION : State.WAITING_FOR_CONTROLLER;
        };
    controllerAlert.set(state == State.WAITING_FOR_CONTROLLER);
    Logger.recordOutput("Choreographer/Goal", goal);
    Logger.recordOutput("Choreographer/State", state);
    return state;
  }

  public boolean canCharacterize() {
    return state == State.CHARACTERIZATION;
  }

  public void enforceOutputInterlocks() {
    if (state == State.IDLE || state == State.WAITING_FOR_CONTROLLER) {
      drive.neutral();
    }
  }
}
