// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package first.robot.util;

import org.wpilib.command3.Mechanism;

/** A subsystem with separate input and output phases owned by RobotContainer. */
public abstract class FullSubsystem implements Mechanism {
  /** Read and log inputs before commands run. */
  public abstract void periodic();

  /** Apply the final output requests after commands and interlocks run. */
  public abstract void periodicAfterScheduler();
}
