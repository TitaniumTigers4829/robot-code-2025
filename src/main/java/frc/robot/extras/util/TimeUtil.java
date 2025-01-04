package frc.robot.extras.util;

import org.littletonrobotics.junction.Logger;

public class TimeUtil {
  /**
   * @return Gets the deterministic timestamp in seconds.
   */
  public static double getLogTimeSeconds() {
    return Logger.getTimestamp() / 1_000_000.0;
  }

  /**
   * @return Gets the non-deterministic timestamp in seconds.
   */
  public static double getRealTimeSeconds() {
    return Logger.getRealTimestamp() / 1_000_000.0;
  }
}
