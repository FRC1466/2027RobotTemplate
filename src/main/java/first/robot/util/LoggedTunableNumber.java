// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package first.robot.util;

import first.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** A configured default with an AdvantageKit-logged override in tuning mode. */
public final class LoggedTunableNumber implements DoubleSupplier {
  private final double defaultValue;
  private final LoggedNetworkNumber tuner;

  public LoggedTunableNumber(String key, double defaultValue) {
    this.defaultValue = defaultValue;
    tuner = Constants.TUNING_MODE ? new LoggedNetworkNumber("/Tuning/" + key, defaultValue) : null;
  }

  public double get() {
    double value = tuner == null ? defaultValue : tuner.get();
    return Double.isFinite(value) ? value : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
