package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import frc.robot.Constants.HardwareConstants;

public class PhysicalGyroPigeon implements GyroInterface {

  private final Pigeon2 gyro = new Pigeon2(0, HardwareConstants.CANIVORE_CAN_BUS_STRING);
  private final Pigeon2Configuration gyroConfiguration;

  public PhysicalGyroPigeon() {
    gyroConfiguration = new Pigeon2Configuration();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    // These values are negated to make sure CCW is positive for our gyro rotation as is consistent
    // with the rest of pose estimation. We do this
    // because sometimes the gyro can be oriented differently, causing the reading to be negative.
    inputs.yawVelocityDegreesPerSecond = gyro.getAngularVelocityZWorld().getValueAsDouble();
    inputs.yawDegrees = gyro.getYaw().getValueAsDouble();
    inputs.accelX = gyro.getAccelerationX().getValueAsDouble();
    inputs.accelY = gyro.getAccelerationY().getValueAsDouble();
    inputs.rollDegrees = gyro.getRoll().getValueAsDouble();
    inputs.pitchDegrees = gyro.getPitch().getValueAsDouble();
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
