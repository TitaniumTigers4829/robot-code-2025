// All praise 254 lib

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionInterface visionInterface;
  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

  public VisionSubsystem(VisionInterface visionInterface) {
    // Initializing Fields
    this.visionInterface = visionInterface;
  }

  @Override
  public void periodic() {
    // Updates limelight inputs
    visionInterface.updateInputs(inputs);
    Logger.processInputs("Vision/", inputs);
  }

  // Add methods to support DriveCommand
  public int getNumberOfAprilTags(Limelight limelight) {
    return inputs.limelightTargets[limelight.getId()];
  }

  public double getLimelightAprilTagDistance(Limelight limelight) {
    return inputs.limelightAprilTagDistances[limelight.getId()];
  }

  public double getTimeStampSeconds(Limelight limelight) {
    return inputs.limelightTimestamps[limelight.getId()];
  }

  public double getLatencySeconds(Limelight limelight) {
    return inputs.limelightLatencies[limelight.getId()];
  }

  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    visionInterface.setHeadingInfo(headingDegrees, headingRateDegrees);
  }

  @AutoLogOutput(key = "Vision/Has Targets")
  public boolean canSeeAprilTags(Limelight limelight) {
    return inputs.limelightSeesAprilTags[limelight.getId()];
  }

  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return inputs.limelightCalculatedPoses[limelight.getId()];
  }

  public Pose2d getLastSeenPose() {
    return visionInterface.getLastSeenPose();
  }
}
