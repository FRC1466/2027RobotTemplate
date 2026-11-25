package first.robot.subsystems.example;

import org.wpilib.math.system.DCMotor;

/**
 * Pivot facts. Every value here is a placeholder for the example. Replace each with a CAD or
 * measured value and note its source, or mark it as an estimate.
 */
public final class ExamplePivotConstants {
  private ExamplePivotConstants() {}

  /** Placeholder: two Kraken X60s on one gearbox, leader first. */
  public static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(2);

  /** Motor names in the order of the per-motor input arrays. */
  public static final String[] MOTOR_NAMES = {"Leader", "Follower"};

  /** Placeholder: motor rotations per pivot rotation. */
  public static final double GEAR_RATIO = 60.0;

  /** Placeholder: soft travel limits. Zero is horizontal, positive is up. */
  public static final double MIN_ANGLE_RAD = Math.toRadians(-30);

  public static final double MAX_ANGLE_RAD = Math.toRadians(110);
  public static final double STOW_ANGLE_RAD = Math.toRadians(90);

  /** Estimate: moving mass and its distance from the pivot, for the simulation only. */
  public static final double MASS_KG = 4.0;

  public static final double PIVOT_TO_MASS_METERS = 0.4;

  public static final double AT_GOAL_TOLERANCE_RAD = Math.toRadians(2);
  public static final double HOT_MOTOR_CELSIUS = 70.0;
}
