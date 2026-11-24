package first.robot;

import org.wpilib.framework.RobotBase;

public final class Constants {
  private Constants() {}

  // Replace the example Tuner constants with verified values before enabling hardware.
  public static final boolean ENABLE_REAL_DRIVE = false;
  public static final boolean TUNING_MODE = false;

  public static final class CheckDeploy {
    public static void main(String... args) {
      if (!ENABLE_REAL_DRIVE) {
        throw new IllegalStateException(
            "SystemCore deploy blocked. Verify the Tuner constants and set ENABLE_REAL_DRIVE in Constants.java.");
      }
    }
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final Mode currentMode =
      RobotBase.isReal()
          ? Mode.REAL
          : System.getenv("AKIT_LOG_PATH") == null ? Mode.SIM : Mode.REPLAY;
}
