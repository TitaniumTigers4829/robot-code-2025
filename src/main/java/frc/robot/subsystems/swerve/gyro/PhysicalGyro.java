package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class PhysicalGyro implements GyroInterface {
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
  private final Queue<Angle> yawPositionInput;

  public PhysicalGyro() {
    yawPositionInput = OdometryThread.registerInput(() -> Degrees.of(gyro.getAngle()));
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    // Handle turn absolute positions
    if (!yawPositionInput.isEmpty()) {
      double gyroAngle = 0.0;
      for (Angle angle : yawPositionInput) {
        gyroAngle = angle.in(Degrees);
      }
      inputs.yawDegrees = gyroAngle;
      yawPositionInput.clear();
    }
    inputs.isConnected = gyro.isConnected();
    inputs.yawVelocity = gyro.getRate();
    inputs.accelX = gyro.getWorldLinearAccelX();
    inputs.accelY = gyro.getWorldLinearAccelY();
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
