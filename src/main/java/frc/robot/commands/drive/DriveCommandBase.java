// TigerLib 2024

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.interpolators.MultiLinearInterpolator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import frc.robot.subsystems.vision.VisionSubsystem;

public abstract class DriveCommandBase extends Command {

  private final MultiLinearInterpolator oneAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  private final VisionSubsystem vision;
  private final SwerveDrive swerveDrive;

  private double lastTimeStampSeconds = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   * @param vision The subsystem for vision measurements
   */
  public DriveCommandBase(SwerveDrive swerveDrive, VisionSubsystem vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    // It is important that you do addRequirements(driveSubsystem, vision) in whatever
    // command extends this
    // DO NOT do addRequirements here, it will break things
  }

  @Override
  public void execute() {
    swerveDrive.addPoseEstimatorSwerveMeasurement();
    vision.setHeadingInfo(
        swerveDrive.getPose().getRotation().getDegrees(), swerveDrive.getGyroRate());
    calculatePoseFromLimelight(Limelight.SHOOTER);
    calculatePoseFromLimelight(Limelight.FRONT_LEFT);
    calculatePoseFromLimelight(Limelight.FRONT_RIGHT);
  }

  public void calculatePoseFromLimelight(Limelight limelightNumber) {
    // Updates the robot's odometry with april tags
    if (vision.canSeeAprilTags(limelightNumber)) {
      double distanceFromClosestAprilTag = vision.getLimelightAprilTagDistance(limelightNumber);

      // Depending on how many april tags we see, we change our confidence as more april tags
      // results in a much more accurate pose estimate
      // So if we only see 1 april tag, we have *high* standard deviations -> lower confidence
      if (vision.getNumberOfAprilTags(limelightNumber) == 1) {
        // But then we use the lookup table here to account for how far away the robot is from the
        // april tag
        // because if we are closer to the april tag, we are more confident in our position -> lower
        // standard deviation
        double[] standardDeviations =
            oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        swerveDrive.setPoseEstimatorVisionConfidence(
            standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      } else if (vision.getNumberOfAprilTags(limelightNumber) > 1) {
        double[] standardDeviations =
            twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        swerveDrive.setPoseEstimatorVisionConfidence(
            standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      }

      // Adds the timestamped pose gotten from the limelights to our pose estimation
      swerveDrive.addPoseEstimatorVisionMeasurement(
          vision.getPoseFromAprilTags(limelightNumber),
          Timer.getFPGATimestamp() - vision.getLatencySeconds(limelightNumber));
    }
  }
}
