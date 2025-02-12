// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowChoreoTrajectory extends DriveCommandBase {
  SwerveDrive swerveDrive;
  SwerveSample swerveSample;

  /** Creates a new FollowChoreoTrajectory. */
  public FollowChoreoTrajectory(SwerveDrive swerveDrive, VisionSubsystem visionSubsystem) {
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive, visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public FollowChoreoTrajectory(
      SwerveDrive swerveDrive, VisionSubsystem visionSubsystem, SwerveSample swerveSample) {
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    this.swerveSample = swerveSample;
    addRequirements(swerveDrive, visionSubsystem);
  }

  public void setSample(SwerveSample sample) {
    swerveSample = sample;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    swerveDrive.followTrajectory(swerveSample);
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
