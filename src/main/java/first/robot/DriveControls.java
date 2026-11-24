package first.robot;

import first.robot.subsystems.Choreographer.State;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCharacterization;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

final class DriveControls {
  private static final double DEADBAND = 0.10;
  private final Drive drive;
  private final DriveCharacterization characterization;
  private final Gamepad controller = new Gamepad(0);
  private final DriverInputsAutoLogged inputs = new DriverInputsAutoLogged();
  private boolean xLockReady;
  private boolean zeroHeadingReady;
  private Rotation2d driverHeadingOffset = Rotation2d.ZERO;

  @AutoLog
  static class DriverInputs {
    public boolean teleopEnabled;
    public boolean testEnabled;
    public boolean connected;
    public double forward;
    public double left;
    public double rotation;
    public boolean xLock;
    public boolean zeroHeading;
    public boolean wheelRadiusTest;
    public boolean feedforwardTest;
  }

  DriveControls(Drive drive) {
    this.drive = drive;
    characterization = new DriveCharacterization(drive);
    controller.setLeftXDeadband(0);
    controller.setLeftYDeadband(0);
    controller.setRightXDeadband(0);
  }

  void updateInputs() {
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      inputs.teleopEnabled = RobotState.isTeleopEnabled();
      inputs.testEnabled = RobotState.isUtilityEnabled();
      inputs.connected = controller.isConnected();
      if (inputs.connected) {
        inputs.forward = -controller.getLeftY();
        inputs.left = -controller.getLeftX();
        inputs.rotation = -controller.getRightX();
        inputs.xLock = controller.getFaceLeftButton();
        inputs.zeroHeading = controller.getBackButton();
        inputs.wheelRadiusTest = controller.getLeftBumperButton();
        inputs.feedforwardTest = controller.getRightBumperButton();
      } else {
        inputs.forward = 0;
        inputs.left = 0;
        inputs.rotation = 0;
        inputs.xLock = false;
        inputs.zeroHeading = false;
        inputs.wheelRadiusTest = false;
        inputs.feedforwardTest = false;
      }
    }
    Logger.processInputs("Drive/Driver", inputs);
  }

  boolean teleopEnabled() {
    return inputs.teleopEnabled;
  }

  boolean connected() {
    return inputs.connected;
  }

  boolean testEnabled() {
    return inputs.testEnabled;
  }

  void characterize(boolean allowed) {
    characterization.periodic(allowed, inputs.wheelRadiusTest, inputs.feedforwardTest);
  }

  void periodic(State state) {
    if (state != State.DRIVER_CONTROL) {
      xLockReady = false;
      zeroHeadingReady = false;
      if (state == State.AUTONOMOUS) {
        driverHeadingOffset = Rotation2d.ZERO;
      }
      return;
    }

    if (!inputs.zeroHeading) {
      zeroHeadingReady = true;
    } else if (zeroHeadingReady) {
      driverHeadingOffset = drive.getRotation();
      zeroHeadingReady = false;
    }

    if (!inputs.xLock) {
      xLockReady = true;
    } else if (xLockReady) {
      drive.stopWithX();
      return;
    }

    var fieldRelative =
        new ChassisVelocities(
            shape(inputs.forward) * drive.getMaxLinearSpeedMetersPerSec(),
            shape(inputs.left) * drive.getMaxLinearSpeedMetersPerSec(),
            shape(inputs.rotation) * drive.getMaxAngularSpeedRadPerSec());
    drive.runVelocity(
        fieldRelative.toRobotRelative(drive.getRotation().minus(driverHeadingOffset)));
  }

  private static double shape(double value) {
    double scaled = MathUtil.applyDeadband(value, DEADBAND);
    return Math.copySign(scaled * scaled, scaled);
  }
}
