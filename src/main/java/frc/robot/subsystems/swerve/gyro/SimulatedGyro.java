package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.mechanismSim.swerve.GyroSimulation;

public class SimulatedGyro implements GyroInterface {
  private final GyroSimulation gyroSimulation;

  public SimulatedGyro(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = true;
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.odometryYawTimestamps = OdometryTimestampsSim.getTimestamps();
    inputs.yawDegreesRotation2d = gyroSimulation.getGyroReading();
    inputs.yawDegrees = gyroSimulation.getGyroReading().getDegrees();
    inputs.yawVelocity =
        RadiansPerSecond.of(gyroSimulation.getMeasuredAngularVelocityRadPerSec())
            .in(DegreesPerSecond);
  }

  @Override
  public void reset() {
    gyroSimulation.setRotation(Rotation2d.kZero);
  }
}
