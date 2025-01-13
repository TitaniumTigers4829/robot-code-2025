package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.extras.sim.SimGyro;

public class SimulatedGyro implements GyroInterface {
  private double yawRads = 0.0;
  private double yawVelRadsPerSec = 0.0;
  private double accelX = 0.0;
  private double accelY = 0.0;

  public SimulatedGyro(SimGyro simGyro) {
    simGyro.setUpdateConsumer((yawPair, accelVector) -> {
      this.yawRads = yawPair.getFirst().in(Degrees);
      this.yawVelRadsPerSec = yawPair.getSecond().in(DegreesPerSecond);
      this.accelX = accelVector.x().in(MetersPerSecondPerSecond);
      this.accelY = accelVector.y().in(MetersPerSecondPerSecond);
    });
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = true;
    inputs.yawDegrees = yawRads;
    inputs.yawVelocity = yawVelRadsPerSec;
    inputs.accelX = accelX;
    inputs.accelY = accelY;
  }

  @Override
  public void reset() {
    yawRads = 0.0;
  }
}
