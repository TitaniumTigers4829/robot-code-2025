package frc.robot.extras.simulation;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.extras.simulation.field.SimulatedField;

public class OdometryTimestampsSim {
  public static double[] getTimestamps() {
    final double[] odometryTimestamps = new double[5];
    for (int i = 0; i < 5; i++)
      odometryTimestamps[i] =
          Timer.getFPGATimestamp() - Robot.defaultPeriodSecs + SimulatedField.SIMULATION_DT * i;
    return odometryTimestamps;
  }
}
