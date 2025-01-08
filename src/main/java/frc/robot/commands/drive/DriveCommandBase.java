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
    calculatePoseFromLimelight(Limelight.BACK);
    calculatePoseFromLimelight(Limelight.FRONT_LEFT);
    calculatePoseFromLimelight(Limelight.FRONT_RIGHT);
  }

  public void calculatePoseFromLimelight(Limelight limelightNumber) {
    double currentTimeStampSeconds = lastTimeStampSeconds;

    // Updates the robot's odometry with april tags
    if (vision.canSeeAprilTags(limelightNumber)) {
      currentTimeStampSeconds = vision.getTimeStampSeconds(limelightNumber);

      double distanceFromClosestAprilTag = vision.getLimelightAprilTagDistance(limelightNumber);

      // Depending on how many april tags we see, we change our confidence as more april tags
      // results in a much more accurate pose estimate
      // TODO: check if this is necessary anymore with MT2, also we might want to set the limelight
      //  so it only uses 1 april tag, if they set up the field wrong (they can set april tags +-1
      // inch I believe)
      //  using multiple *could* really mess things up.
      if (vision.getNumberOfAprilTags(limelightNumber) == 1) {
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

      swerveDrive.addPoseEstimatorVisionMeasurement(
          vision.getPoseFromAprilTags(limelightNumber),
          Timer.getFPGATimestamp() - vision.getLatencySeconds(limelightNumber));
    }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }
}
