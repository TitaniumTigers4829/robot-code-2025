package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class PhysicalGyro implements GyroInterface {

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);

  public PhysicalGyro() {}
  

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    // These values are negated to make sure CCW is positive for our gyro rotation as is consistent
    // with the rest of pose estimation. We do this
    // because sometimes the gyro can be oriented differently, causing the reading to be negative.
    inputs.yawVelocityDegreesPerSecond = -gyro.getRate();
    inputs.yawDegrees = -gyro.getAngle();
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
