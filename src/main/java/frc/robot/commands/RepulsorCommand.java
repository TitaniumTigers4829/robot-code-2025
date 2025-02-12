// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RepulsorCommand extends DriveCommandBase {
  private SwerveDrive swerveDrive;
  private final Pose2d setpoint;
  private final Pose2d estimatedPose;

  /** Creates a new RepulsorCommand. */
  public RepulsorCommand(
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      Pose2d setpoint,
      Pose2d estimatedPose) {
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    this.setpoint = setpoint;
    this.estimatedPose = estimatedPose;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.autoAlign(new Pose3d(setpoint), estimatedPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
