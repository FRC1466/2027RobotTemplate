package first.robot.subsystems.example;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

/**
 * Roller model. While a simulated piece is in the rollers and they intake, the piece jams them: the
 * rollers stop and the motor draws stall current, limited like a Talon FX stator current limit.
 * Motor, reduction, and inertia are placeholders.
 */
public class ExampleRollersIOSim implements ExampleRollersIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);
  private static final double GEAR_RATIO = 3.0;
  private static final double MOI = 0.002;
  private static final double STATOR_LIMIT_AMPS = 60.0;
  private static final double AMBIENT_CELSIUS = 25.0; // Temperature is not modeled.

  private final DCMotorSim sim =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(GEARBOX, MOI, GEAR_RATIO), GEARBOX);
  private double appliedVolts = 0.0;
  private boolean pieceInRollers = false;

  /** Put a simulated game piece in the rollers, or take it out. */
  public void setPieceInRollers(boolean pieceInRollers) {
    this.pieceInRollers = pieceInRollers;
  }

  @Override
  public void updateInputs(ExampleRollersIOInputs inputs) {
    boolean jammed = pieceInRollers && appliedVolts > 0.0;
    sim.setInputVoltage(jammed ? 0.0 : appliedVolts);
    sim.update(0.02);
    if (jammed) {
      sim.setAngularVelocity(0.0);
    }

    // Stator current from the motor's voltage equation at its actual speed.
    double motorRadPerSec = sim.getAngularVelocity() * GEAR_RATIO;
    double statorAmps = (appliedVolts - motorRadPerSec / GEARBOX.Kv) / GEARBOX.R;
    inputs.connected = true;
    inputs.velocityRotPerSec = sim.getAngularVelocity() / (2 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.min(Math.abs(statorAmps), STATOR_LIMIT_AMPS);
    inputs.tempCelsius = AMBIENT_CELSIUS;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = Math.clamp(volts, -12.0, 12.0);
  }
}
