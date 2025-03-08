package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.AquilaConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.DevConstants;

/** Swerve Constants */
public class SwerveConstants {

  public static final class DriveConstants {
    public static final double X_POS_TRUST = 0.03; // Meters
    public static final double Y_POS_TRUST = 0.03; // Meters
    public static final double ANGLE_TRUST = Units.degreesToRadians(1); // Radians

    // Wheel base and track width are measured by the center of the swerve modules, not the frame of
    // the robot
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH =
        Constants.getRobot() == RobotType.DEV_ROBOT
            ? Units.inchesToMeters(22)
            : Units.inchesToMeters(21.25);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE =
        Constants.getRobot() == RobotType.DEV_ROBOT
            ? Units.inchesToMeters(22)
            : Units.inchesToMeters(21.25);
    public static final double DRIVE_BASE_DIAMETER =
        Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
        };

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    public static final class AquilaConstants {
      public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 22;
      public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 24;
      public static final int REAR_LEFT_DRIVE_MOTOR_ID = 23;
      public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 21;

      public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
      public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
      public static final int REAR_LEFT_TURN_MOTOR_ID = 8;
      public static final int REAR_RIGHT_TURN_MOTOR_ID = 7;

      public static final int FRONT_LEFT_CANCODER_ID = 14;
      public static final int FRONT_RIGHT_CANCODER_ID = 12;
      public static final int REAR_LEFT_CANCODER_ID = 11;
      public static final int REAR_RIGHT_CANCODER_ID = 13;

      public static final double FRONT_LEFT_ZERO_ANGLE = 0.137939453125;
      public static final double FRONT_RIGHT_ZERO_ANGLE = -0.420654296875;
      public static final double REAR_LEFT_ZERO_ANGLE = -0.475341796875;
      public static final double REAR_RIGHT_ZERO_ANGLE = -0.05078125;

      public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;

      public static final InvertedValue FRONT_LEFT_DRIVE_ENCODER_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_DRIVE_ENCODER_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_LEFT_DRIVE_ENCODER_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_DRIVE_ENCODER_REVERSED =
          InvertedValue.CounterClockwise_Positive;
    }

    public static final class DevConstants {
      public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
      public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
      public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
      public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;

      public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
      public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
      public static final int REAR_LEFT_TURN_MOTOR_ID = 7;
      public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

      public static final int FRONT_LEFT_CANCODER_ID = 9;
      public static final int FRONT_RIGHT_CANCODER_ID = 10;
      public static final int REAR_LEFT_CANCODER_ID = 11;
      public static final int REAR_RIGHT_CANCODER_ID = 12;

      public static final double FRONT_LEFT_ZERO_ANGLE = -0.09521484375;
      public static final double FRONT_RIGHT_ZERO_ANGLE = -0.478271484375;
      public static final double REAR_LEFT_ZERO_ANGLE = -0.318115234375;
      public static final double REAR_RIGHT_ZERO_ANGLE = -0.473388671875;

      public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;

      public static final InvertedValue FRONT_LEFT_DRIVE_ENCODER_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_DRIVE_ENCODER_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_DRIVE_ENCODER_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_DRIVE_ENCODER_REVERSED =
          InvertedValue.CounterClockwise_Positive;
    }

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 20;
    public static final double LOW_ANGULAR_SPEED_RADIANS_PER_SECOND = 5;

    public static final double MAX_SPEED_METERS_PER_SECOND =
        Constants.getRobot() == RobotType.DEV_ROBOT ? 4.5 : 6.95; // 4.5
  }

  public class ModuleConstants {

    public static final double GYRO_MAX_PITCH = 30.0; // degrees
    public static final double GYRO_MAX_ROLL = 30.0; // degrees
    
    public static final double DRIVE_GEAR_RATIO =
        Constants.getRobot() == RobotType.DEV_ROBOT ? 7.13 : 4.59; // 4.59
    public static final double TURN_GEAR_RATIO = 11.3142;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.9);

    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_TO_METERS_PER_SECOND =
        WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

    public static final double DRIVE_SUPPLY_LIMIT = 55.0;
    public static final double DRIVE_STATOR_LIMIT = 60.0;

    public static final double TURN_P = 1000.0;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 25.0;

    public static final double TURN_S = 0.0;
    public static final double TURN_V = 0.0;
    public static final double TURN_A = 0.0;

    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 30;
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 24;

    public static final double DRIVE_P = 6.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    // These values were characterized using our characterization commands.
    public static final double DRIVE_S = .151315113225759;
    public static final double DRIVE_V = 0.272854272591;
    public static final double DRIVE_A = 0.0;
  }

  public static final ModuleConfig[] aquilaModuleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            AquilaConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            AquilaConstants.FRONT_LEFT_TURN_MOTOR_ID,
            AquilaConstants.FRONT_LEFT_CANCODER_ID,
            AquilaConstants.FRONT_LEFT_ZERO_ANGLE,
            AquilaConstants.FRONT_LEFT_CANCODER_REVERSED,
            AquilaConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            AquilaConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            AquilaConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            AquilaConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            AquilaConstants.FRONT_RIGHT_CANCODER_ID,
            AquilaConstants.FRONT_RIGHT_ZERO_ANGLE,
            AquilaConstants.FRONT_RIGHT_CANCODER_REVERSED,
            AquilaConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            AquilaConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            AquilaConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            AquilaConstants.REAR_LEFT_TURN_MOTOR_ID,
            AquilaConstants.REAR_LEFT_CANCODER_ID,
            AquilaConstants.REAR_LEFT_ZERO_ANGLE,
            AquilaConstants.REAR_LEFT_CANCODER_REVERSED,
            AquilaConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            AquilaConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            AquilaConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            AquilaConstants.REAR_RIGHT_TURN_MOTOR_ID,
            AquilaConstants.REAR_RIGHT_CANCODER_ID,
            AquilaConstants.REAR_RIGHT_ZERO_ANGLE,
            AquilaConstants.REAR_RIGHT_CANCODER_REVERSED,
            AquilaConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            AquilaConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED)
      };

  public static final ModuleConfig[] devModuleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            DevConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            DevConstants.FRONT_LEFT_TURN_MOTOR_ID,
            DevConstants.FRONT_LEFT_CANCODER_ID,
            DevConstants.FRONT_LEFT_ZERO_ANGLE,
            DevConstants.FRONT_LEFT_CANCODER_REVERSED,
            DevConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            DevConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DevConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            DevConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            DevConstants.FRONT_RIGHT_CANCODER_ID,
            DevConstants.FRONT_RIGHT_ZERO_ANGLE,
            DevConstants.FRONT_RIGHT_CANCODER_REVERSED,
            DevConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            DevConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DevConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            DevConstants.REAR_LEFT_TURN_MOTOR_ID,
            DevConstants.REAR_LEFT_CANCODER_ID,
            DevConstants.REAR_LEFT_ZERO_ANGLE,
            DevConstants.REAR_LEFT_CANCODER_REVERSED,
            DevConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            DevConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DevConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            DevConstants.REAR_RIGHT_TURN_MOTOR_ID,
            DevConstants.REAR_RIGHT_CANCODER_ID,
            DevConstants.REAR_RIGHT_ZERO_ANGLE,
            DevConstants.REAR_RIGHT_CANCODER_REVERSED,
            DevConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            DevConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED)
      };

  public static final ModuleConfig[] compModuleConfigs = null;

  public record ModuleConfig(
      int driveMotorChannel,
      int turnMotorChannel,
      int turnEncoderChannel,
      double angleZero,
      SensorDirectionValue encoderReversed,
      InvertedValue turnReversed,
      InvertedValue driveReversed) {}
}
