package first.robot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Loop timing, command activity, and battery health for AdvantageScope and Driver Station. */
final class RobotDiagnostics {
  private static final double LOW_BATTERY_VOLTS = 11.0;
  private static final double DISABLED_DELAY_SECONDS = 2.0;

  @AutoLog
  static class BatteryInputs {
    public double voltage = 0.0;
    public boolean enabled = false;
  }

  private final BatteryInputsAutoLogged battery = new BatteryInputsAutoLogged();
  private final Alert batteryAlert =
      new Alert("LowBattery", "Battery voltage is low while disabled.", Level.MEDIUM);
  private double disabledSince = Double.NaN;
  private long lastPhaseNanos;

  void beginLoop() {
    lastPhaseNanos = System.nanoTime();
  }

  void recordPhase(String phase) {
    long now = System.nanoTime();
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      Logger.recordOutput("LoggedTracer/" + phase + "MS", (now - lastPhaseNanos) / 1e6);
    }
    lastPhaseNanos = now;
  }

  void periodic(Scheduler scheduler) {
    Logger.recordOutput(
        "Commands/Active",
        scheduler.getRunningCommands().stream().map(Command::name).sorted().toArray(String[]::new));

    if (Constants.currentMode != Constants.Mode.REPLAY) {
      battery.voltage = RobotController.getBatteryVoltage();
      battery.enabled = RobotState.isEnabled();
    }
    Logger.processInputs("Diagnostics/Battery", battery);
    double now = Timer.getTimestamp();
    if (battery.enabled) {
      disabledSince = Double.NaN;
    } else if (Double.isNaN(disabledSince)) {
      disabledSince = now;
    }
    boolean batteryLow =
        now - disabledSince >= DISABLED_DELAY_SECONDS
            && battery.voltage > 0.0
            && battery.voltage <= LOW_BATTERY_VOLTS;
    batteryAlert.set(batteryLow);
    Logger.recordOutput("Diagnostics/Battery/LowWhileDisabled", batteryLow);
  }
}
