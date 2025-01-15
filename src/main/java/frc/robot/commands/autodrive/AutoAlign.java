// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  private final SwerveDrive swerveDrive;
  private Pose2d reef;
  private Pose2d processor;
  private Pose2d feederStation;
   private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          SwerveConstants.AUTO_LINEUP_REEF_ROTATION_P,
          SwerveConstants.AUTO_LINEUP_REEF_ROTATION_I,
          SwerveConstants.AUTO_LINEUP_REEF_ROTATION_D,
          SwerveConstants.AUTO_LINEUP_REEF_ROTATION_CONSTRAINTS;

          SwerveConstants.AUTO_LINEUP_PROCESSOR_ROTATION_P,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_ROTATION_I,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_ROTATION_D,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_ROTATION_CONSTRAINTS;

          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_ROTATION_P,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_ROTATION_I,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_ROTATION_D,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_ROTATION_CONSTRAINTS);

  private final ProfiledPIDController xTranslationController =
      new ProfiledPIDController(
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_P,
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_I,
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_D,
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_CONSTRAINTS;

          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_P,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_I,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_D,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_CONSTRAINTS;

          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_P,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_I,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_D,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController yTranslationController =
      new ProfiledPIDController(
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_P,
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_I,
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_D,
          SwerveConstants.AUTO_LINEUP_REEF_TRANSLATION_CONSTRAINTS;

          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_P,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_I,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_D,
          SwerveConstants.AUTO_LINEUP_PROCESSOR_TRANSLATION_CONSTRAINTS;

          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_P,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_I,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_D,
          SwerveConstants.AUTO_LINEUP_FEEDER_STATION_TRANSLATION_CONSTRAINTS);
  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveDrive swerveDrive, VisionSubsystem visionSubsystem) {
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
