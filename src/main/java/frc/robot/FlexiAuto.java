package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;

public class FlexiAuto {
  private final AutoFactory autoFactory;

  public FlexiAuto(AutoFactory autoFactory) {
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
    return routine;
  }
}
