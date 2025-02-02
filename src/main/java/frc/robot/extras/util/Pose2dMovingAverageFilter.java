package frc.robot.extras.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A moving average filter for smoothing Pose2d values.
 */
public class Pose2dMovingAverageFilter {
  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private final LinearFilter thetaFilter;

  private double filteredX = 0.0;
  private double filteredY = 0.0;
  private double filteredTheta = 0.0;

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
   * Adds a new pose to the filter and updates the moving average.
   * 
   * @param rawPose The new pose to be added.
   */
  public void addPose(Pose2d rawPose) {
    filteredX = xFilter.calculate(rawPose.getX());
    filteredY = yFilter.calculate(rawPose.getY());
    filteredTheta = thetaFilter.calculate(rawPose.getRotation().getRadians());
  }

  /**
   * Returns the current averaged pose.
   * 
   * @return The averaged Pose2d.
   */
  public Pose2d getAverage() {
    return new Pose2d(filteredX, filteredY, new Rotation2d(filteredTheta));
  }
}
