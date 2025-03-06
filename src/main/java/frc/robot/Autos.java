package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Autos {
  private final AutoFactory autoFactory;
  private final SwerveDrive swerveDrive;
  private final VisionSubsystem visionSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  public Autos(
      AutoFactory autoFactory,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem) {
    this.autoFactory = autoFactory;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
  }

  public AutoRoutine oneMeterTestAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.ONE_METER_AUTO_ROUTINE);
    AutoTrajectory oneMeterTrajectory = routine.trajectory(AutoConstants.ONE_METER_TRAJECTORY);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.ONE_METER_TRAJECTORY),
                oneMeterTrajectory.cmd()));
    return routine;
  }

  public AutoRoutine exampleAutoRoutine() {

    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.EXAMPLE_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj = routine.trajectory(AutoConstants.E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory cToPickupTraj = routine.trajectory(AutoConstants.C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_C_TRAJECTORY);

    // reset odometry and start first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));

    return routine;
  }

  public AutoRoutine oneCoralAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.ONE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj = routine.trajectory(AutoConstants.E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory cToPickupTraj = routine.trajectory(AutoConstants.C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_C_TRAJECTORY);

    // reset odometry and start first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj
        .done()
        .onTrue(
            elevatorSubsystem
                .setElevationPosition(ElevatorSetpoints.L4.getPosition())
                .andThen(coralIntakeSubsystem.ejectCoral()));

    return routine;
  }
}
