// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignWithProcessor extends Command {
  /** Creates a new AutoAlignWithProcessor. */

  private final SwerveDrive swerveDrive;
  private Pose2d processorPose;
    private final ProfiledPIDController turnControllerProcessor =
      new ProfiledPIDController(
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_ROTATION_P,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_ROTATION_I,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_ROTATION_D,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_ROTATION_CONSTRAINTS);

    private final ProfiledPIDController xTranslationControllerProcessor =
      new ProfiledPIDController(
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_P,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_I,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_D,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_CONSTRAINTS);

    private final ProfiledPIDController yTranslationControllerProcessor =
      new ProfiledPIDController(
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_P,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_I,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_D,
          SwerveConstants.TrajectoryConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_CONSTRAINTS);
  
  public AutoAlignWithProcessor(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Pose2d(
        FieldConstants.RED_PROCESSOR_PLACE_X,
        FieldConstants.RED_PROCESSOR_PLACE_Y,
        FieldConstants.RED_PROCESSOR_ROTATION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d drivePose = swerveDrive.getPose();
    double xPoseError = processorPose.getX() - drivePose.getX();
    double yPoseError = processorPose.getY() - drivePose.getY();
    double thetaPoseError =
        processorPose.getRotation().getRadians() - drivePose.getRotation().getRadians();

    // Uses the PID controllers to calculate the drive output
    double xOutput = deadband(xTranslationControllerProcessor.calculate(xPoseError, 0));
    double yOutput = deadband(yTranslationControllerProcessor.calculate(yPoseError, 0));
    double turnOutput = deadband(turnControllerProcessor.calculate(thetaPoseError, 0));

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

  private double deadband(double value) {
    double deadband = 0.05; // Example deadband value
    if (Math.abs(value) > deadband) {
      return value;
    } else {
      return 0.0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
