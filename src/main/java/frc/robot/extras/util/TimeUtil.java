package frc.robot.extras.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;

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
    return RobotController.getFPGATime() / 1_000_000.0;
  }
}
