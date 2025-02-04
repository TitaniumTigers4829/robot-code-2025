package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autos {
  private final AutoFactory autoFactory;
  private final ExampleSubsystem exampleSubsystem;
  private final SwerveDrive swerveDrive;

  public Autos(
      AutoFactory autoFactory, ExampleSubsystem exampleSubsystem, SwerveDrive swerveDrive) {
    this.autoFactory = autoFactory;
    this.exampleSubsystem = exampleSubsystem;
    this.swerveDrive = swerveDrive;
  }

  public AutoRoutine exampleAutoRoutine() {

    AutoRoutine routine = autoFactory.newRoutine("exampleAutoRoutine");

    AutoTrajectory startToETraj = routine.trajectory("startToE");
    AutoTrajectory eToPickupTraj = routine.trajectory("eToPickup");
    AutoTrajectory cToPickupTraj = routine.trajectory("cToPickup");
    AutoTrajectory pickupToCTraj = routine.trajectory("pickupToC");

    // reset odometry and start first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(autoFactory.resetOdometry("Right-Start-To-E"), startToETraj.cmd()));

    startToETraj
        .active()
        .onTrue(
            exampleSubsystem
                .exampleFunctionalCommand()); // TODO: replace with elevator to L4 command
    startToETraj
        .atTime("score")
        .onTrue(
            exampleSubsystem.exampleFunctionalCommand()); // TODO: replace with command for rollers
    startToETraj
        .done()
        .onTrue(
            eToPickupTraj
                .cmd()
                .alongWith(
                    exampleSubsystem
                        .exampleFunctionalCommand())); // TODO: replace with elevator to intake
    // command

    return routine;
  }
}
