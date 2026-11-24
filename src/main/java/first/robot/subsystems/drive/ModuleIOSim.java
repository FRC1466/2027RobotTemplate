// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in AdvantageKit-License.md
// at the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.util.Nat;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared
  // locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models
    driveSim = createMotorSim(DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio);
    turnSim = createMotorSim(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static DCMotorSim createMotorSim(DCMotor motor, double inertia, double reduction) {
    // States and outputs are mechanism position and velocity. The input is motor voltage.
    double voltageGain = reduction * motor.Kt / (motor.R * inertia);
    double velocityGain = -voltageGain * reduction / motor.Kv;
    return new DCMotorSim(
        new LinearSystem<>(
            MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0, velocityGain),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0, voltageGain),
            MatBuilder.fill(Nat.N2(), Nat.N2(), 1, 0, 0, 1),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0, 0)),
        motor);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveSim.getInputVoltage();
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnSim.getInputVoltage();
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDraw());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
