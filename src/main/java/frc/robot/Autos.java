package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.example.ExampleSubsystem;

public class Autos {
  public AutoFactory autoFactory;

  public AutoRoutine exampleAutoRoutine(ExampleSubsystem exampleSubsystem) {

    AutoRoutine routine = autoFactory.newRoutine("exampleAutoRoutine");

    AutoTrajectory startToETraj = routine.trajectory("startToE");
    AutoTrajectory eToPickupTraj = routine.trajectory("eToPickup");
    AutoTrajectory cToPickupTraj = routine.trajectory("cToPickup");
    AutoTrajectory pickupToCTraj = routine.trajectory("pickupToC");

    // reset odometry and start first trajectory
    routine.active().onTrue(Commands.sequence(startToETraj.resetOdometry(), startToETraj.cmd()));

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
