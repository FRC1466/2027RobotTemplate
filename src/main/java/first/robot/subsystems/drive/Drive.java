// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in AdvantageKit-License.md
// at the root directory of this project.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;

import first.robot.Constants;
import first.robot.Constants.Mode;
import first.robot.generated.TunerConstants;
import first.robot.util.FullSubsystem;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Drive extends FullSubsystem {
  // TunerConstants doesn't include these constants, so they are declared locally
  static double odometryFrequency() {
    return TunerConstants.canBus().isNetworkFD() ? 250.0 : 100.0;
  }

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("SwerveGyro", "Disconnected gyro, using kinematics as fallback.", Level.HIGH);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO);
  private final SwerveDriveOdometry simOdometry =
      Constants.currentMode == Mode.SIM
          ? new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO)
          : null;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Start odometry thread
    if (gyroIO instanceof GyroIOPigeon2) {
      PhoenixOdometryThread.getInstance().start();
    }
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (var module : modules) {
      sampleCount = Math.min(sampleCount, module.getOdometryPositions().length);
    }
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      if (simOdometry != null) {
        simOdometry.update(rawGyroRotation, modulePositions);
      }
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  @Override
  public void periodicAfterScheduler() {
    double statorCurrentAmps = 0;
    for (var module : modules) {
      module.applyOutputs();
      statorCurrentAmps += module.getStatorCurrentAmps();
    }
    Logger.recordOutput("Diagnostics/Drive/TotalStatorCurrentAmps", statorCurrentAmps);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities discreteSpeeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteSpeeds);
    setpointStates =
        SwerveDriveKinematics.desaturateWheelVelocities(
            setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    var optimizedSetpoints = new SwerveModuleVelocity[modules.length];
    for (int i = 0; i < modules.length; i++) {
      optimizedSetpoints[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log the optimized requests sent to the modules.
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpoints);
  }

  /** Clears motor requests when the drive has no operator. */
  public void neutral() {
    for (var module : modules) {
      module.stop();
    }
    Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[0]);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[0]);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", new ChassisVelocities());
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    var locations = kinematics.getModules();
    Rotation2d[] headings = new Rotation2d[locations.length];
    for (int i = 0; i < locations.length; i++) {
      headings[i] = locations[i].getAngle().orElseThrow();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  void orientForWheelRadiusTest() {
    var locations = kinematics.getModules();
    Rotation2d[] headings = new Rotation2d[locations.length];
    for (int i = 0; i < locations.length; i++) {
      headings[i] = locations[i].getAngle().orElseThrow().plus(Rotation2d.CCW_PI_2);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  boolean usesDriveVoltageControl() {
    for (var module : modules) {
      if (!module.usesDriveVoltageControl()) {
        return false;
      }
    }
    return true;
  }

  void requestCharacterizationVoltage(double volts) {
    for (var module : modules) {
      module.requestCharacterizationVoltage(volts);
    }
  }

  DriveCharacterization.ModuleSample[] getCharacterizationSamples() {
    var samples = new DriveCharacterization.ModuleSample[modules.length];
    for (int i = 0; i < modules.length; i++) {
      samples[i] = modules[i].getCharacterizationSample();
    }
    return samples;
  }

  boolean hasCharacterizationGyro() {
    return gyroInputs.connected && Double.isFinite(gyroInputs.yawPosition.getRadians());
  }

  Rotation2d getCharacterizationYaw() {
    return gyroInputs.yawPosition;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }

  /** Places the simulated robot at the authored start and resets the drive estimate. */
  public void setAutoStartPose(Pose2d pose) {
    if (simOdometry != null) {
      simOdometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }
    setPose(pose);
  }

  /** Wheel-only simulated pose used to generate camera frames. */
  public Pose2d getSimPose() {
    return simOdometry.getPose();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
