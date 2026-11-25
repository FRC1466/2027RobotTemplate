package first.robot.subsystems.example;

import org.littletonrobotics.junction.AutoLog;

/**
 * Pivot hardware boundary. Angles are pivot angles above horizontal, already converted from motor
 * rotations. Apply the gear ratio exactly once, here in the IO, in real and simulation IO alike.
 */
public interface ExamplePivotIO {
  @AutoLog
  public static class ExamplePivotIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    /** The profiled reference the controller is tracking this loop (Motion Magic reference). */
    public double setpointRad = 0.0;

    /** Per motor, in {@link ExamplePivotConstants#MOTOR_NAMES} order. */
    public boolean[] connected = new boolean[] {};

    public double[] appliedVolts = new double[] {};
    public double[] statorCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(ExamplePivotIOInputs inputs) {}

  /** Profiled move to the angle goal (Motion Magic on Talon FX hardware). */
  default void runPosition(double positionRad) {}

  /** Stop driving the motors. The next {@link #runPosition} starts from the measured state. */
  default void neutral() {}

  default void configure(
      double kP, double kD, double cruiseRadPerSec, double accelerationRadPerSecSq) {}
}
