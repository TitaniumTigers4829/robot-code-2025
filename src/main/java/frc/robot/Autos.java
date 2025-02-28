package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;

public class Autos {
  private final AutoFactory autoFactory;

  public Autos(AutoFactory autoFactory) {
    this.autoFactory = autoFactory;
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
    oneMeterTrajectory
        .done()
        .onTrue(new RunCommand(() -> SmartDashboard.putBoolean("Trajectory Done", true)));
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
    startToETraj.done().onTrue(eToPickupTraj.cmd());

    return routine;
  }

  public AutoRoutine twoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj = routine.trajectory(AutoConstants.E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_C_TRAJECTORY);

    // reset odometry and start first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());

    return routine;
  }

  public AutoRoutine threeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj = routine.trajectory(AutoConstants.E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj = routine.trajectory(AutoConstants.C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_D_TRAJECTORY);

    // reset odometry and start first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().onTrue(pickupToDTraj.cmd());

    return routine;
  }
}
