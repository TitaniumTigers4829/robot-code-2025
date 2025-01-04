package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/** Swerve Constants */
public class SwerveConstants {

  public static final class DriveConstants {
    public static final double X_POS_TRUST = 0.03; // Meters
    public static final double Y_POS_TRUST = 0.03; // Meters
    public static final double ANGLE_TRUST = Units.degreesToRadians(1); // Radians

    // Wheel base and track width are measured by the center of the swerve modules, not the frame of
    // the robot
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.25);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(21.25);

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
        };

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(MODULE_TRANSLATIONS);

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

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 20;
    public static final double LOW_ANGULAR_SPEED_RADIANS_PER_SECOND = 5;

    public static final double MAX_SPEED_METERS_PER_SECOND = 6.95;
    public static final double MAX_SHOOT_SPEED_METERS_PER_SECOND = 3;

    public static final double HEADING_ACCEPTABLE_ERROR_RADIANS = Units.degreesToRadians(2.5);
    public static final double HEADING_ACCEPTABLE_ERROR_MOVING_RADIANS = Units.degreesToRadians(4);
  }

  public class ModuleConstants {
    public static final double DRIVE_GEAR_RATIO = 4.59;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.774788522800778);

    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_TO_METERS_PER_SECOND =
        WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

    public static final double DRIVE_SUPPLY_LIMIT = 45.0;
    public static final double DRIVE_STATOR_LIMIT = 50.0;

    public static final double TURN_P = 116;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.64;

    public static final double TURN_S = 0.0;
    public static final double TURN_V = 0.0;
    public static final double TURN_A = 0.0;

    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 30;
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 24;

    public static final double DRIVE_P = 0.417;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    public static final double DRIVE_S = 0.16;
    // These values were gotten using recalc, then converted to the correct units & were confirmed
    // through testing and characterization
    // https://www.reca.lc/drive?appliedVoltageRamp=%7B%22s%22%3A1200%2C%22u%22%3A%22V%2Fs%22%7D&batteryAmpHours=%7B%22s%22%3A18%2C%22u%22%3A%22A%2Ah%22%7D&batteryResistance=%7B%22s%22%3A0.018%2C%22u%22%3A%22Ohm%22%7D&batteryVoltageAtRest=%7B%22s%22%3A12.6%2C%22u%22%3A%22V%22%7D&efficiency=97&filtering=1&gearRatioMax=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&gearRatioMin=%7B%22magnitude%22%3A3%2C%22ratioType%22%3A%22Reduction%22%7D&maxSimulationTime=%7B%22s%22%3A4%2C%22u%22%3A%22s%22%7D&maxSpeedAccelerationThreshold=%7B%22s%22%3A0.15%2C%22u%22%3A%22ft%2Fs2%22%7D&motor=%7B%22quantity%22%3A4%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&motorCurrentLimit=%7B%22s%22%3A60%2C%22u%22%3A%22A%22%7D&numCyclesPerMatch=24&peakBatteryDischarge=20&ratio=%7B%22magnitude%22%3A4.59%2C%22ratioType%22%3A%22Reduction%22%7D&sprintDistance=%7B%22s%22%3A25%2C%22u%22%3A%22ft%22%7D&swerve=1&targetTimeToGoal=%7B%22s%22%3A2%2C%22u%22%3A%22s%22%7D&throttleResponseMax=0.99&throttleResponseMin=0.5&weightAuxilliary=%7B%22s%22%3A24%2C%22u%22%3A%22lbs%22%7D&weightDistributionFrontBack=0.5&weightDistributionLeftRight=0.5&weightInspected=%7B%22s%22%3A125%2C%22u%22%3A%22lbs%22%7D&wheelBaseLength=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D&wheelBaseWidth=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&wheelCOFDynamic=0.9&wheelCOFLateral=1.1&wheelCOFStatic=1.1&wheelDiameter=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D
    public static final double DRIVE_V =
        1.73 * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO; // = 0.1203 V*s/m
    public static final double DRIVE_A =
        0.32 * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO; // = 0.02225 V*s^2/m
  }

  public static final class TrajectoryConstants {

    public static final double DRIVE_BASE_DIAMETER =
        Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));

    public static final double MAX_SPEED = 5.0;
    public static final double MAX_ACCELERATION = 3;

    public static final double AUTO_TRANSLATION_P = 1.5; // 1.7
    public static final double AUTO_TRANSLATION_D = 0.2;
    public static final double AUTO_THETA_P = 4.5; // 5
    public static final double AUTO_THETA_D = 0.4;

    public static final double AUTO_SHOOT_HEADING_OFFSET = 2;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;

    // Note Detection Driving Constants
    public static final double AUTO_ALIGN_TRANSLATIONAL_P = 3;
    public static final double AUTO_ALIGN_TRANSLATIONAL_I = 0;
    public static final double AUTO_ALIGN_TRANSLATIONAL_D = 0;

    public static Constraints AUTO_ALIGN_TRANSLATION_CONSTRAINTS = new Constraints(5, 2);

    public static final double AUTO_ALIGN_ROTATIONAL_P = 3;
    public static final double AUTO_ALIGN_ROTATIONAL_I = 0;
    public static final double AUTO_ALIGN_ROTATIONAL_D = 0;

    public static Constraints AUTO_ALIGN_ROTATIONAL_CONSTRAINTS =
        new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2);
  }

  public static final ModuleConfig[] moduleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
            DriveConstants.FRONT_LEFT_CANCODER_ID,
            DriveConstants.FRONT_LEFT_ZERO_ANGLE,
            DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            DriveConstants.FRONT_RIGHT_CANCODER_ID,
            DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
            DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
            DriveConstants.REAR_LEFT_CANCODER_ID,
            DriveConstants.REAR_LEFT_ZERO_ANGLE,
            DriveConstants.REAR_LEFT_CANCODER_REVERSED,
            DriveConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
            DriveConstants.REAR_RIGHT_CANCODER_ID,
            DriveConstants.REAR_RIGHT_ZERO_ANGLE,
            DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
            DriveConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED)
      };

  public record ModuleConfig(
      int driveMotorChannel,
      int turnMotorChannel,
      int turnEncoderChannel,
      double angleZero,
      SensorDirectionValue encoderReversed,
      InvertedValue turnReversed,
      InvertedValue driveReversed) {}
}
