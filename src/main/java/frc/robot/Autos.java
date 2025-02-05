package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.Constants.AutoConstants;

public class Autos {
  private final AutoFactory autoFactory;
  private final SwerveDrive swerveDrive;

  public Autos(
      AutoFactory autoFactory, SwerveDrive swerveDrive) {
    this.autoFactory = autoFactory;
    this.swerveDrive = swerveDrive;
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
            Commands.sequence(autoFactory.resetOdometry(AutoConstants.RIGHT_START_TO_E_TRAJECTORY), startToETraj.cmd()));

    startToETraj
        .active()
        .onTrue(); // TODO: replace with elevator to L4 command
    startToETraj
        .atTime("score")
        .onTrue(
    startToETraj
        .done()
        .onTrue(
            eToPickupTraj
                .cmd()));
    // command

    return routine;
  }
}
