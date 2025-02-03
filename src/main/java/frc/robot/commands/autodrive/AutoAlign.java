// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends DriveCommandBase {

  private final SwerveDrive swerveDrive;
  private final VisionSubsystem visionSubsystem;

  private Pose2d Pose;

  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_ROTATION_P,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_ROTATION_I,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_ROTATION_D,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_ROTATION_CONSTRAINTS);

  private final ProfiledPIDController xTranslationController =
      new ProfiledPIDController(
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_P,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_I,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_D,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController yTranslationController =
      new ProfiledPIDController(
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_P,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_I,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_D,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_TRANSLATION_CONSTRAINTS);

  /**
   * Creates a new AutoAlign.
   *
   * @param visionSubsystem The subsystem for vision
   * @param swerveDrive The subsystem for the swerve drive
   * @param targetPose The target pose for the robot to align to
   */
  public AutoAlign(SwerveDrive swerveDrive, VisionSubsystem visionSubsystem, Pose2d targetPose) {
    super(swerveDrive, visionSubsystem);
    this.Pose = targetPose;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, visionSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    // Gets the error between the desired pos (the target) and the current pos of the robot
    Pose2d drivePose = swerveDrive.getEstimatedPose();
    double xPoseError = Pose.getX() - drivePose.getX();
    double yPoseError = Pose.getY() - drivePose.getY();
    double thetaPoseError = Pose.getRotation().getRadians() - drivePose.getRotation().getRadians();

    // Uses the PID controllers to calculate the drive output
    double xOutput = MathUtil.applyDeadband(xTranslationController.calculate(xPoseError, 0), 0.05);
    double yOutput = MathUtil.applyDeadband(yTranslationController.calculate(yPoseError, 0), 0.05);
    double turnOutput =
        MathUtil.applyDeadband(rotationController.calculate(thetaPoseError, 0), 0.05);

    // Enables continuous input for the rotation controller
    rotationController.enableContinuousInput(0, 2 * Math.PI);

    // Gets the chassis speeds for the robot using the odometry rotation (not alliance relative)
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput, yOutput, turnOutput, swerveDrive.getOdometryRotation2d());

    // Drives the robot towards the amp
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
