package first.robot.subsystems.example;

import static first.robot.subsystems.example.ExamplePivotConstants.*;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Point-mass pivot model with an onboard profile standing in for Motion Magic. SingleJointedArmSim
 * models a uniform rod, so a rod 1.5x the pivot-to-mass distance gives the point mass's gravity
 * torque. The feedforward uses the same model, so simulated tracking is better than real hardware.
 */
public class ExamplePivotIOSim implements ExamplePivotIO {
  private static final double DT = 0.02;
  private static final double AMBIENT_CELSIUS = 25.0; // Temperature is not modeled.
  private static final double MOI = MASS_KG * PIVOT_TO_MASS_METERS * PIVOT_TO_MASS_METERS;
  private static final double GRAVITY_VOLTS_HORIZONTAL =
      GEARBOX.getVoltage(MASS_KG * 9.81 * PIVOT_TO_MASS_METERS / GEAR_RATIO, 0);
  private static final double KV = GEAR_RATIO / GEARBOX.Kv;
  private static final double KA = GEARBOX.getVoltage(MOI / GEAR_RATIO, 0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX,
          GEAR_RATIO,
          MOI,
          1.5 * PIVOT_TO_MASS_METERS,
          MIN_ANGLE_RAD,
          MAX_ANGLE_RAD,
          true,
          STOW_ANGLE_RAD);
  private final PIDController feedback = new PIDController(0, 0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private boolean closedLoop = false;
  private double goalRad = STOW_ANGLE_RAD;

  @Override
  public void updateInputs(ExamplePivotIOInputs inputs) {
    double volts = 0.0;
    if (closedLoop) {
      var previous = setpoint;
      setpoint = profile.calculate(DT, setpoint, new TrapezoidProfile.State(goalRad, 0));
      double acceleration = (setpoint.velocity - previous.velocity) / DT;
      volts =
          GRAVITY_VOLTS_HORIZONTAL * Math.cos(setpoint.position)
              + KV * setpoint.velocity
              + KA * acceleration
              + feedback.calculate(sim.getAngle(), setpoint.position);
    }
    volts = Math.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(volts);
    sim.update(DT);

    inputs.positionRad = sim.getAngle();
    inputs.velocityRadPerSec = sim.getVelocity();
    inputs.setpointRad = closedLoop ? setpoint.position : sim.getAngle();
    // The gearbox model reports total current; split it evenly between the motors.
    double current = Math.abs(sim.getCurrentDraw()) / MOTOR_NAMES.length;
    inputs.connected = new boolean[MOTOR_NAMES.length];
    inputs.appliedVolts = new double[MOTOR_NAMES.length];
    inputs.statorCurrentAmps = new double[MOTOR_NAMES.length];
    inputs.tempCelsius = new double[MOTOR_NAMES.length];
    for (int i = 0; i < MOTOR_NAMES.length; i++) {
      inputs.connected[i] = true;
      inputs.appliedVolts[i] = volts;
      inputs.statorCurrentAmps[i] = current;
      inputs.tempCelsius[i] = AMBIENT_CELSIUS;
    }
  }

  @Override
  public void runPosition(double positionRad) {
    if (!closedLoop) {
      setpoint = new TrapezoidProfile.State(sim.getAngle(), sim.getVelocity());
      feedback.reset();
    }
    closedLoop = true;
    goalRad = positionRad;
  }

  @Override
  public void neutral() {
    closedLoop = false;
  }

  @Override
  public void configure(
      double kP, double kD, double cruiseRadPerSec, double accelerationRadPerSecSq) {
    feedback.setPID(kP, 0, kD);
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(cruiseRadPerSec, accelerationRadPerSecSq));
  }
}
