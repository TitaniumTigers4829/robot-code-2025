package frc.robot.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class PhysicalGyro implements GyroInterface {

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);

  public PhysicalGyro() {}

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    inputs.yawDegreesRotation2d = gyro.getRotation2d();
    inputs.yawVelocityDegreesPerSecond = gyro.getRate();
    inputs.yawDegrees = gyro.getAngle();
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
