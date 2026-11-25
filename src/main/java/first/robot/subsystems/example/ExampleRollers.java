package first.robot.subsystems.example;

import first.robot.util.FullSubsystem;
import first.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Intake rollers that detect a game piece from motor current. Runs only in the loops where a caller
 * requests intake or eject.
 *
 * <p>While intaking, a piece jams the rollers, so stator current stays high while roller speed
 * drops. Spinning up also draws high current at low speed, so the first moments after intake starts
 * are ignored, and the stall must last a short confirm time. Ejecting long enough clears the piece.
 * Detection uses only logged inputs and replay-aware time, so it replays. The thresholds are
 * estimates until measured with real game pieces.
 */
public class ExampleRollers extends FullSubsystem {
  public enum Request {
    STOP,
    INTAKE,
    EJECT
  }

  private static final double HOT_MOTOR_CELSIUS = 70.0;

  private final ExampleRollersIO io;
  private final ExampleRollersIOInputsAutoLogged inputs = new ExampleRollersIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("ExampleRollersDisconnected", "Roller motor disconnected.", Level.MEDIUM);
  private final Alert hotAlert =
      new Alert("ExampleRollersHot", "Roller motor above 70 C.", Level.MEDIUM);
  private final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("ExampleRollers/IntakeVolts", 8.0);
  private final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("ExampleRollers/EjectVolts", -6.0);

  private final LoggedTunableNumber pieceCurrentAmps =
      new LoggedTunableNumber("ExampleRollers/PieceCurrentAmps", 30.0);
  private final LoggedTunableNumber pieceMaxVelocity =
      new LoggedTunableNumber("ExampleRollers/PieceMaxVelocityRotPerSec", 2.0);
  private final LoggedTunableNumber spinUpIgnoreSeconds =
      new LoggedTunableNumber("ExampleRollers/SpinUpIgnoreSeconds", 0.25);
  private final LoggedTunableNumber pieceConfirmSeconds =
      new LoggedTunableNumber("ExampleRollers/PieceConfirmSeconds", 0.1);
  private final LoggedTunableNumber ejectClearSeconds =
      new LoggedTunableNumber("ExampleRollers/EjectClearSeconds", 0.25);

  private Request request = Request.STOP;
  private Request applied = Request.STOP;
  private double appliedSince = 0.0;
  private double stalledSince = Double.NaN;
  private boolean hasGamePiece = false;

  public ExampleRollers(ExampleRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ExampleRollers", inputs);
    disconnectedAlert.set(!inputs.connected);
    hotAlert.set(inputs.tempCelsius > HOT_MOTOR_CELSIUS);
    updateDetection();
  }

  /** Inputs reflect last loop's output, so detection judges the request that was applied. */
  private void updateDetection() {
    double now = Timer.getTimestamp();
    boolean stalled =
        inputs.connected
            && inputs.statorCurrentAmps > pieceCurrentAmps.get()
            && Math.abs(inputs.velocityRotPerSec) < pieceMaxVelocity.get();
    switch (applied) {
      case INTAKE -> {
        if (stalled && now - appliedSince >= spinUpIgnoreSeconds.get()) {
          if (Double.isNaN(stalledSince)) {
            stalledSince = now;
          }
          if (now - stalledSince >= pieceConfirmSeconds.get()) {
            hasGamePiece = true;
          }
        } else {
          stalledSince = Double.NaN;
        }
      }
      case EJECT -> {
        stalledSince = Double.NaN;
        if (now - appliedSince >= ejectClearSeconds.get()) {
          hasGamePiece = false;
        }
      }
      case STOP -> stalledSince = Double.NaN;
    }
    Logger.recordOutput("ExampleRollers/HasGamePiece", hasGamePiece);
    Logger.recordOutput("Diagnostics/ExampleRollers/Stalled", stalled);
  }

  /** Whether a game piece has been detected and not ejected since. */
  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  /** Request roller output for this loop only. */
  public void request(Request request) {
    this.request = request;
  }

  @Override
  public void periodicAfterScheduler() {
    double volts =
        switch (request) {
          case STOP -> 0.0;
          case INTAKE -> intakeVolts.get();
          case EJECT -> ejectVolts.get();
        };
    io.setVoltage(volts);
    if (request != applied) {
      applied = request;
      appliedSince = Timer.getTimestamp();
    }
    Logger.recordOutput("ExampleRollers/Request", request);
    Logger.recordOutput("ExampleRollers/RequestedVolts", volts);
    Logger.recordOutput("Diagnostics/ExampleRollers/StatorCurrentAmps", inputs.statorCurrentAmps);
    request = Request.STOP;
  }
}
