// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;

public class PathPlannerAutoDrive extends DriveCommandBase {
  /** Creates a new PathPlannerAutoDrive. */
  private final SwerveDrive swerveDrive;

  private final VisionSubsystem visionSubsystem;
  private Pose2d goalPose;
  private PPHolonomicDriveController driveController = AutoConstants.PP_AUTO_ALIGN_DRIVE_CONTROLLER;

  public PathPlannerAutoDrive(
      SwerveDrive swerveDrive, VisionSubsystem visionSubsystem, Pose2d goalPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    addRequirements(swerveDrive, visionSubsystem);
    Pose2d currentPose = swerveDrive.getEstimatedPose();
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                currentPose.getTranslation(),
                getPathVelocityHeading(swerveDrive.getChassisSpeeds(), goalPose)),
            new Pose2d(
                new Translation2d(
                    currentPose.getTranslation().getX() + 1, currentPose.getTranslation().getY()),
                getPathVelocityHeading(swerveDrive.getChassisSpeeds(), goalPose)));
    // goalPose);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            AutoConstants.PP_AUTO_ALIGN_CONSTRAINTS,
            new IdealStartingState(
                getVelocityMagnitude(swerveDrive.getChassisSpeeds()),
                swerveDrive.getEstimatedPose().getRotation()),
            new GoalEndState(
                0.0,
                goalPose.getRotation()) // Goal end state. You can set a holonomic rotation here.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalPose = goalState.pose;
    SmartDashboard.putString("Pose", goalPose.toString());
    SmartDashboard.putString("Actual Pose", swerveDrive.getEstimatedPose().toString());
    swerveDrive.drive(
        driveController
            .calculateRobotRelativeSpeeds(swerveDrive.getEstimatedPose(), goalState)
            .unaryMinus());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * @param cs field relative chassis speeds
   * @return
   */
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
      var diff = target.minus(swerveDrive.getEstimatedPose()).getTranslation();
      return (diff.getNorm() < 0.01)
          ? target.getRotation()
          : diff.getAngle(); // .rotateBy(Rotation2d.k180deg);
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
    return MetersPerSecond.of(
        new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }
}
