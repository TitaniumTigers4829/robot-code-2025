package frc.robot.extras.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pose2dMovingAverageFilter {
  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private final LinearFilter theataFilter;

  public Pose2dMovingAverageFilter(int windowSize) {
    xFilter = LinearFilter.movingAverage(windowSize);
    yFilter = LinearFilter.movingAverage(windowSize);
    theataFilter = LinearFilter.movingAverage(windowSize);
  }

  public Pose2d calculate(Pose2d rawPose) {
    double filteredX = xFilter.calculate(rawPose.getX());
    double filteredY = yFilter.calculate(rawPose.getY());
    double filteredTheta = theataFilter.calculate(rawPose.getRotation().getRadians());

    return new Pose2d(filteredX, filteredY, new Rotation2d(filteredTheta));
  }
}
