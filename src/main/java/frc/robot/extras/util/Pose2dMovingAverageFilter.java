package frc.robot.extras.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A moving average filter for smoothing Pose2d values. This filter averages the X, Y, and rotation
 * components separately over a given window size.
 */
public class Pose2dMovingAverageFilter {
  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private final LinearFilter thetaFilter;

  /**
   * Constructs a Pose2dMovingAverageFilter with a specified window size.
   *
   * @param windowSize The number of samples to average.
   */
  public Pose2dMovingAverageFilter(int windowSize) {
    xFilter = LinearFilter.movingAverage(windowSize);
    yFilter = LinearFilter.movingAverage(windowSize);
    thetaFilter = LinearFilter.movingAverage(windowSize);
  }

  /**
   * Applies the moving average filter to a given Pose2d.
   *
   * @param rawPose The input pose to be filtered.
   * @return A new Pose2d representing the filtered (smoothed) position and rotation.
   */
  public Pose2d calculate(Pose2d rawPose) {
    double filteredX = xFilter.calculate(rawPose.getX());
    double filteredY = yFilter.calculate(rawPose.getY());
    double filteredTheta = thetaFilter.calculate(rawPose.getRotation().getRadians());

    return new Pose2d(filteredX, filteredY, new Rotation2d(filteredTheta));
  }
}
