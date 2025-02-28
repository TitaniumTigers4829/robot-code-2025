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

  // Blue Auto Routines
  public AutoRoutine blueOneMeterTestAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_ONE_METER_AUTO_ROUTINE);
    AutoTrajectory oneMeterTrajectory = routine.trajectory(AutoConstants.BLUE_ONE_METER_TRAJECTORY);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_ONE_METER_TRAJECTORY),
                oneMeterTrajectory.cmd()));
    oneMeterTrajectory
        .done()
        .onTrue(new RunCommand(() -> SmartDashboard.putBoolean("Trajectory Done", true)));
    return routine;
  }

  public AutoRoutine blueTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());

    return routine;
  }

  public AutoRoutine blueThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().onTrue(pickupToDTraj.cmd());

    return routine;
  }

  public AutoRoutine blueFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToBTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_B_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().onTrue(pickupToDTraj.cmd());
    pickupToDTraj.done().onTrue(dToPickupTraj.cmd());
    dToPickupTraj.done().onTrue(pickupToBTraj.cmd());

    return routine;
  }

  // Red Auto Routines
  public AutoRoutine redOneMeterTestAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_ONE_METER_AUTO_ROUTINE);
    AutoTrajectory oneMeterTrajectory = routine.trajectory(AutoConstants.RED_ONE_METER_TRAJECTORY);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_ONE_METER_TRAJECTORY),
                oneMeterTrajectory.cmd()));
    oneMeterTrajectory
        .done()
        .onTrue(new RunCommand(() -> SmartDashboard.putBoolean("Trajectory Done", true)));
    return routine;
  }

  public AutoRoutine redTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());

    return routine;
  }

  public AutoRoutine redThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.RED_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().onTrue(pickupToDTraj.cmd());

    return routine;
  }

  public AutoRoutine redFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.RED_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.RED_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToBTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_B_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().onTrue(pickupToDTraj.cmd());
    pickupToDTraj.done().onTrue(dToPickupTraj.cmd());
    dToPickupTraj.done().onTrue(pickupToBTraj.cmd());

    return routine;
  }
}
