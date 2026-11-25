package first.robot.subsystems;

import first.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Resolves robot-level goals into subsystem ownership and stop interlocks.
 *
 * <h2>Mechanism ownership</h2>
 *
 * Each state has one owner: the driver in driver control, autonomous in autonomous, and nobody
 * otherwise. Mechanism requests carry their {@link Source} and are refused unless that source owns
 * the current state, so a routine cannot move a mechanism in teleop and a held button does nothing
 * in autonomous. Driver controls use the same calls as autonomous.
 *
 * <p>To add a mechanism, keep its request here as an {@link OwnedRequest}, expose a source-checked
 * method, and pass the value to the mechanism in {@link #enforceOutputInterlocks}:
 *
 * <pre>{@code
 * private final OwnedRequest<Double> pivotGoalRad = held(ExamplePivotConstants.STOW_ANGLE_RAD);
 * private final OwnedRequest<Rollers.Request> rollerRequest = latched(Rollers.Request.STOP);
 *
 * public boolean requestPivot(Source source, double angleRad) {
 *   return pivotGoalRad.request(source, angleRad);
 * }
 *
 * public boolean requestRollers(Source source, Rollers.Request request) {
 *   return rollerRequest.request(source, request);
 * }
 *
 * // In enforceOutputInterlocks(), every loop:
 * pivot.setGoal(pivotGoalRad.get());
 * if (owner() != null) {
 *   rollers.request(rollerRequest.get());
 * }
 * }</pre>
 *
 * <p>Driver controls call {@code requestPivot(Source.DRIVER, ...)} and routines call {@code
 * requestPivot(Source.AUTO, ...)}. A refused request returns false; a routine can treat that as a
 * fault.
 *
 * <p>A {@link #held} request, such as a position goal, stays until someone requests another, even
 * across a change of owner. A {@link #latched} request, such as rollers left running, returns to
 * its stopped value whenever the owner changes, so it never outlives the source that asked for it.
 */
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

  /** Who is asking for a mechanism action. Only the owner of the current state is obeyed. */
  public enum Source {
    DRIVER,
    AUTO
  }

  /**
   * A mechanism request that only the current owner can change. Create one with {@link #held} or
   * {@link #latched}.
   */
  public final class OwnedRequest<T> {
    private final T stopped;
    private final boolean resetOnOwnerChange;
    private T value;

    private OwnedRequest(T initial, boolean resetOnOwnerChange) {
      this.stopped = initial;
      this.resetOnOwnerChange = resetOnOwnerChange;
      this.value = initial;
    }

    /**
     * @return false, and nothing changes, if {@code source} does not own the current state
     */
    public boolean request(Source source, T value) {
      if (source != owner()) {
        return false;
      }
      this.value = value;
      return true;
    }

    public T get() {
      return value;
    }
  }

  private final Drive drive;
  private final List<OwnedRequest<?>> requests = new ArrayList<>();
  private Source lastOwner;
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
    var owner = owner();
    if (owner != lastOwner) {
      for (var request : requests) {
        if (request.resetOnOwnerChange) {
          reset(request);
        }
      }
      lastOwner = owner;
    }
    Logger.recordOutput("Choreographer/Owner", owner == null ? "None" : owner.name());
    Logger.recordOutput("Choreographer/Goal", goal);
    Logger.recordOutput("Choreographer/State", state);
    return state;
  }

  /** The source that owns mechanisms in the current state, or null when nobody does. */
  public Source owner() {
    return switch (state) {
      case DRIVER_CONTROL -> Source.DRIVER;
      case AUTONOMOUS -> Source.AUTO;
      case IDLE, WAITING_FOR_CONTROLLER, CHARACTERIZATION -> null;
    };
  }

  /** A request that keeps its value across a change of owner, such as a position goal. */
  public <T> OwnedRequest<T> held(T initial) {
    var request = new OwnedRequest<>(initial, false);
    requests.add(request);
    return request;
  }

  /**
   * A request that returns to {@code stopped} whenever the owner changes, such as rollers a routine
   * leaves running while it drives.
   */
  public <T> OwnedRequest<T> latched(T stopped) {
    var request = new OwnedRequest<>(stopped, true);
    requests.add(request);
    return request;
  }

  private static <T> void reset(OwnedRequest<T> request) {
    request.value = request.stopped;
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
