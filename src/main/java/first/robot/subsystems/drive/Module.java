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
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private SwerveModuleVelocity requestedVelocity;
  private boolean characterizationRequested;
  private double characterizationVolts;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "SwerveDrive" + index, "Disconnected drive motor on module " + index + ".", Level.HIGH);
    turnDisconnectedAlert =
        new Alert(
            "SwerveTurn" + index, "Disconnected turn motor on module " + index + ".", Level.HIGH);
    turnEncoderDisconnectedAlert =
        new Alert(
            "SwerveEncoder" + index,
            "Disconnected turn encoder on module " + index + ".",
            Level.HIGH);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount =
        Math.min(
            inputs.odometryTimestamps.length,
            Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length));
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Stages the optimized velocity request and returns it for logging. */
  public SwerveModuleVelocity runSetpoint(SwerveModuleVelocity state) {
    characterizationRequested = false;
    requestedVelocity = state.optimize(getAngle()).cosineScale(getAngle());
    return requestedVelocity;
  }

  /** Requests neutral outputs. */
  public void stop() {
    requestedVelocity = null;
    characterizationRequested = false;
  }

  boolean usesDriveVoltageControl() {
    return constants.DriveMotorClosedLoopOutput
        == SwerveModuleConstants.ClosedLoopOutputType.Voltage;
  }

  /** Called only for modules configured for voltage output. */
  void requestCharacterizationVoltage(double volts) {
    requestedVelocity = null;
    characterizationRequested = true;
    characterizationVolts = volts;
  }

  void applyOutputs() {
    if (characterizationRequested) {
      io.setDriveOpenLoop(characterizationVolts);
      io.setTurnPosition(Rotation2d.ZERO);
    } else if (requestedVelocity == null) {
      io.setDriveOpenLoop(0.0);
      io.setTurnOpenLoop(0.0);
    } else {
      io.setDriveVelocity(requestedVelocity.velocity / constants.WheelRadius);
      io.setTurnPosition(requestedVelocity.angle);
    }
  }

  double getStatorCurrentAmps() {
    return Math.abs(inputs.driveCurrentAmps) + Math.abs(inputs.turnCurrentAmps);
  }

  DriveCharacterization.ModuleSample getCharacterizationSample() {
    return new DriveCharacterization.ModuleSample(
        inputs.driveConnected && inputs.turnConnected && inputs.turnEncoderConnected,
        inputs.turnPosition,
        inputs.drivePositionRad,
        inputs.driveVelocityRadPerSec / (2 * Math.PI),
        inputs.driveAppliedVolts);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}
