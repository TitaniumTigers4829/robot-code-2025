// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/* Auto Align takes in Pose2d and moves robot to it */
public class AutoAlign extends DriveCommandBase {

  private final SwerveDrive swerveDrive;
  private final VisionSubsystem visionSubsystem;

  private Pose2d targetPose;

  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          AutoConstants.AUTO_ALIGN_ROTATION_P,
          AutoConstants.AUTO_ALIGN_ROTATION_I,
          AutoConstants.AUTO_ALIGN_ROTATION_D,
          AutoConstants.AUTO_ALIGN_ROTATION_CONSTRAINTS);

  private final ProfiledPIDController xTranslationController =
      new ProfiledPIDController(
          AutoConstants.AUTO_ALIGN_TRANSLATION_P,
          AutoConstants.AUTO_ALIGN_TRANSLATION_I,
          AutoConstants.AUTO_ALIGN_TRANSLATION_D,
          AutoConstants.AUTO_ALIGN_TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController yTranslationController =
      new ProfiledPIDController(
          AutoConstants.AUTO_ALIGN_TRANSLATION_P,
          AutoConstants.AUTO_ALIGN_TRANSLATION_I,
          AutoConstants.AUTO_ALIGN_TRANSLATION_D,
          AutoConstants.AUTO_ALIGN_TRANSLATION_CONSTRAINTS);

  /**
   * Creates a new AutoAlign.
   *
   * @param visionSubsystem The subsystem for vision
   * @param swerveDrive The subsystem for the swerve drive
   * @param targetPose The target pose for the robot to align to
   */
  public AutoAlign(SwerveDrive swerveDrive, VisionSubsystem visionSubsystem, Pose2d targetPose) {
    super(swerveDrive, visionSubsystem);
    this.targetPose = targetPose;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, visionSubsystem);
    // Enables continuous input for the rotation controller
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    // Gets the error between the desired pos (the target) and the current pos of the robot
    Pose2d drivePose = swerveDrive.getEstimatedPose();
    double xPoseError = targetPose.getX() - drivePose.getX();
    double yPoseError = targetPose.getY() - drivePose.getY();
    double thetaPoseError =
        targetPose.getRotation().getRadians() - drivePose.getRotation().getRadians();

    // Uses the PID controllers to calculate the drive output
    double xOutput =
        MathUtil.applyDeadband(
            xTranslationController.calculate(xPoseError, 0),
            AutoConstants.TRANSLATION_DEADBAND_AMOUNT);
    double yOutput =
        MathUtil.applyDeadband(
            yTranslationController.calculate(yPoseError, 0),
            AutoConstants.TRANSLATION_DEADBAND_AMOUNT);
    double turnOutput =
        MathUtil.applyDeadband(
            rotationController.calculate(thetaPoseError, 0),
            AutoConstants.TRANSLATION_DEADBAND_AMOUNT);

    // Gets the chassis speeds for the robot using the odometry rotation (not alliance relative)
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput, yOutput, turnOutput, swerveDrive.getOdometryRotation2d());

    // Drives the robot towards the target pose
    swerveDrive.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
