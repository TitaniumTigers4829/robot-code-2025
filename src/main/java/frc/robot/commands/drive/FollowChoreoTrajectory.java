package frc.robot.commands.drive;

import choreo.trajectory.SwerveSample;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowChoreoTrajectory extends DriveCommandBase {
  private final SwerveDrive swerveDrive;
  private SwerveSample sample;

  // Constructor without TrajectorySample for AutoFactory
  public FollowChoreoTrajectory(
      SwerveDrive swerveDrive, VisionSubsystem vision, SwerveSample sample) {
    super(swerveDrive, vision);
    this.swerveDrive = swerveDrive;
    this.sample = sample;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sample != null) {
      swerveDrive.followTrajectory(sample);
    }
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
