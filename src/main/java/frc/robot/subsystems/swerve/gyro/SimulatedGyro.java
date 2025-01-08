package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.extras.sim.SimGyro;

public class SimulatedGyro implements GyroInterface {

  public SimulatedGyro(SimGyro simGyro) {
    simGyro.setUpdateConsumer((yawPair, accelVector) -> {
      super.yawRads = yawPair.getFirst().in(Radians);
      super.yawVelRadsPerSec = yawPair.getSecond().in(RadiansPerSecond);
      super.accelX = accelVector.x().in(MetersPerSecondPerSecond);
      super.accelY = accelVector.y().in(MetersPerSecondPerSecond);
    });
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = true;
    inputs.odometryYawPositions = SimGyro.getCachedGyroReadings();
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
