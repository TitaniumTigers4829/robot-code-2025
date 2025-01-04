package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import java.util.Queue;

public class PhysicalGyro implements GyroInterface {
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
  private final Queue<Angle> yawPositionInput;

  public PhysicalGyro() {
    yawPositionInput = OdometryThread.registerInput(() -> Degrees.of(gyro.getAngle()));
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    inputs.yawDegreesRotation2d = gyro.getRotation2d();
    inputs.yawVelocity = gyro.getRate();
    inputs.yawDegrees = gyro.getAngle();

    // Handle odometry yaw positions
    if (!yawPositionInput.isEmpty()) {
      Rotation2d[] odometryYawPositions = new Rotation2d[yawPositionInput.size()];
      int index = 0;
      for (Angle angle : yawPositionInput) {
        odometryYawPositions[index++] = Rotation2d.fromDegrees(angle.in(Degrees));
      }
      inputs.odometryYawPositions = odometryYawPositions;
      yawPositionInput.clear();
    }
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
