package first.robot.subsystems.example;

import org.littletonrobotics.junction.AutoLog;

/** Roller hardware boundary. Velocity is roller-side, after the gear ratio. */
public interface ExampleRollersIO {
  @AutoLog
  public static class ExampleRollersIOInputs {
    public boolean connected = false;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(ExampleRollersIOInputs inputs) {}

  default void setVoltage(double volts) {}
}
