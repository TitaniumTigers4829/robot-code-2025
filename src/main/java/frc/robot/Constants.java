package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.*;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

public final class Constants {

  public static final class LogPaths {
    public static final String SYSTEM_PERFORMANCE_PATH = "SystemPerformance/";
    public static final String PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/";
    public static final String APRIL_TAGS_VISION_PATH = "Vision/AprilTags/";
  }

  public static final Mode CURRENT_MODE = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * This is where we place constants related to hardware on a robot that aren't specific to any
   * singular subsystem.
   */
  public static final class HardwareConstants {
    public static final double TIMEOUT_S = 0.02;

    public static final double SIGNAL_FREQUENCY = 250;

    public static final String CANIVORE_CAN_BUS_STRING = "canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    /**
     * For some reason, falcons normally have a deadband threshold of 4%. This is incredibly high!
     * It makes it very hard to do precise movements, so with this constant we set the threshold to
     * the lowest possible value.
     */
    public static final double MIN_FALCON_DEADBAND = 0.001;
  }

  /**
   * stores the constants and PID configs for chassis because we want an all-real simulation for the
   * chassis, the numbers are required to be considerably precise
   */
  public class SimulationConstants {
    /**
     * numbers that needs to be changed to fit each robot TODO: change these numbers to match your
     * robot
     */
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 0.95,
        ROBOT_MASS_KG = 60; // with bumpers

    /** TODO: change motor type to match your robot */
    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1),
        STEER_MOTOR = DCMotor.getFalcon500(1);

    public static final double WHEEL_RADIUS_METERS = ModuleConstants.WHEEL_DIAMETER_METERS / 2.0,
        DRIVE_GEAR_RATIO = ModuleConstants.DRIVE_GEAR_RATIO,
        STEER_GEAR_RATIO = 11.0,
        TIME_ROBOT_STOP_ROTATING_SECONDS = 0.06,
        STEER_FRICTION_VOLTAGE = 0.12,
        DRIVE_FRICTION_VOLTAGE = ModuleConstants.DRIVE_S,
        DRIVE_INERTIA = 0.01,
        STEER_INERTIA = 0.01;

    /* adjust current limit */
    public static final double DRIVE_CURRENT_LIMIT = ModuleConstants.DRIVE_STATOR_LIMIT;
    // public static final double STEER_CURRENT_LIMIT = ModuleConstants.ST;

    /* equations that calculates some constants for the simulator (don't modify) */
    private static final double GRAVITY_CONSTANT = 9.81;
    public static final double DRIVE_BASE_RADIUS = DriveConstants.MODULE_TRANSLATIONS[0].getNorm(),
        /* friction_force = normal_force * coefficient_of_friction */
        MAX_FRICTION_ACCELERATION = GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION,
        MAX_FRICTION_FORCE_PER_MODULE =
            MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG / DriveConstants.MODULE_TRANSLATIONS.length,
        /* force = torque / distance */
        MAX_PROPELLING_FORCE_NEWTONS =
            DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS,
        /* floor_speed = wheel_angular_velocity * wheel_radius */
        CHASSIS_MAX_VELOCITY =
            DRIVE_MOTOR.freeSpeedRadPerSec
                / DRIVE_GEAR_RATIO
                * WHEEL_RADIUS_METERS, // calculate using choreo
        CHASSIS_MAX_ACCELERATION_MPS_SQ =
            Math.min(
                MAX_FRICTION_ACCELERATION, // cannot exceed max friction
                MAX_PROPELLING_FORCE_NEWTONS / ROBOT_MASS_KG),
        CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = CHASSIS_MAX_VELOCITY / DRIVE_BASE_RADIUS,
        CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
            CHASSIS_MAX_ACCELERATION_MPS_SQ / DRIVE_BASE_RADIUS,
        CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION =
            CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC / TIME_ROBOT_STOP_ROTATING_SECONDS;

    /* for collision detection in simulation */
    public static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(34.5),
        BUMPER_LENGTH_METERS = Units.inchesToMeters(36),
        /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
        BUMPER_COEFFICIENT_OF_FRICTION = 0.75,
        /* https://simple.wikipedia.org/wiki/Coefficient_of_restitution */
        BUMPER_COEFFICIENT_OF_RESTITUTION = 0.08;

    /* Gyro Sim */
    public static final double GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ = 100;
    public static final double SKIDDING_AMOUNT_AT_THRESHOLD_RAD = Math.toRadians(1.2);
    /*
     * https://store.ctr-electronics.com/pigeon-2/
     * for a well-installed one with vibration reduction, only 0.4 degree
     * but most teams just install it directly on the rigid chassis frame (including my team :D)
     * so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
     * which is the average velocity during normal swerve-circular-offense
     * */
    public static final double NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD = Math.toRadians(1.2);
    public static final double AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST = Math.toRadians(60);

    public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
    public static final double SIMULATED_PERIOD_SECONDS = 0.02;
  }

  /**
   * This is where constants used to describe the game's field go. This will have the dimensions of
   * the field, but also the coordinates of obstacles, game pieces, or other places of interest.
   */
  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(690);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317);

    public static final double REEF_LEVEL_ONE_Z = Units.inchesToMeters(18);
    public static final double REEF_LEVEL_TWO_Z = Units.inchesToMeters(31.875);
    public static final double REEF_LEVEL_THREE_Z = Units.inchesToMeters(47.625);
    public static final double REEF_LEVEL_FOUR_Z = Units.inchesToMeters(72);

    public static final double CORAL_STATION_Z = Units.inchesToMeters(37.5);

    public static final double PROCESSOR_Z = Units.inchesToMeters(7);

    /* The meaning behind the numbering of the reefs is that it starts from the reef closest to the driver station (Blue side)
    or the reef closer to the robots (Red side) and goes clockwise. Now thinking about it, I should probably find a better way
    to number these reefs so they have some sense on how they're numbered, but that's for other members to decide.
    */
    public static final Pose2d BLUE_REEF_ONE =
        new Pose2d(
            Units.inchesToMeters(155.975),
            Units.inchesToMeters(152.032),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d BLUE_REEF_TWO =
        new Pose2d(
            Units.inchesToMeters(156.02),
            Units.inchesToMeters(164.97),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d BLUE_REEF_THREE =
        new Pose2d(
            Units.inchesToMeters(160.776),
            Units.inchesToMeters(173.226),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d BLUE_REEF_FOUR =
        new Pose2d(
            Units.inchesToMeters(172.003),
            Units.inchesToMeters(179.655),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d BLUE_REEF_FIVE =
        new Pose2d(
            Units.inchesToMeters(181.511),
            Units.inchesToMeters(179.655),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d BLUE_REEF_SIX =
        new Pose2d(
            Units.inchesToMeters(192.693),
            Units.inchesToMeters(173.266),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d BLUE_REEF_SEVEN =
        new Pose2d(
            Units.inchesToMeters(197.515),
            Units.inchesToMeters(165.362),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_REEF_EIGHT =
        new Pose2d(
            Units.inchesToMeters(197.47),
            Units.inchesToMeters(152.03),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_REEF_NINE =
        new Pose2d(
            Units.inchesToMeters(192.714),
            Units.inchesToMeters(143.774),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d BLUE_REEF_TEN =
        new Pose2d(
            Units.inchesToMeters(181.487),
            Units.inchesToMeters(137.345),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d BLUE_REEF_ELEVEN =
        new Pose2d(
            Units.inchesToMeters(137.345),
            Units.inchesToMeters(137.345),
            new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d BLUE_REEF_TWELEVE =
        new Pose2d(
            Units.inchesToMeters(160.797),
            Units.inchesToMeters(143.814),
            new Rotation2d(Units.degreesToRadians(240)));

    public static final Pose2d RED_REEF_ONE =
        new Pose2d(
            Units.inchesToMeters(493.365),
            Units.inchesToMeters(152.032),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_REEF_TWO =
        new Pose2d(
            Units.inchesToMeters(493.41),
            Units.inchesToMeters(164.97),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_REEF_THREE =
        new Pose2d(
            Units.inchesToMeters(498.156),
            Units.inchesToMeters(173.226),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d RED_REEF_FOUR =
        new Pose2d(
            Units.inchesToMeters(509.383),
            Units.inchesToMeters(179.655),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d RED_REEF_FIVE =
        new Pose2d(
            Units.inchesToMeters(518.901),
            Units.inchesToMeters(179.693),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RED_REEF_SIX =
        new Pose2d(
            Units.inchesToMeters(530.083),
            Units.inchesToMeters(173.186),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RED_REEF_SEVEN =
        new Pose2d(
            Units.inchesToMeters(534.895),
            Units.inchesToMeters(164.968),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d RED_REEF_EIGHT =
        new Pose2d(
            Units.inchesToMeters(534.85),
            Units.inchesToMeters(152.03),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d RED_REEF_NINE =
        new Pose2d(
            Units.inchesToMeters(530.104),
            Units.inchesToMeters(143.774),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d RED_REEF_TEN =
        new Pose2d(
            Units.inchesToMeters(518.877),
            Units.inchesToMeters(137.345),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d RED_REEF_ELEVEN =
        new Pose2d(
            Units.inchesToMeters(509.359),
            Units.inchesToMeters(137.307),
            new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d RED_REEF_TWELEVE =
        new Pose2d(
            Units.inchesToMeters(498.177),
            Units.inchesToMeters(143.814),
            new Rotation2d(Units.degreesToRadians(240)));

    public static final Pose2d BLUE_LEFT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(25.8),
            new Rotation2d(Units.degreesToRadians(54)));
    public static final Pose2d BLUE_RIGHT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(291.2),
            new Rotation2d(Units.degreesToRadians(306)));
    public static final Pose2d RED_LEFT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(657.37),
            Units.inchesToMeters(291.2),
            new Rotation2d(Units.degreesToRadians(234)));
    public static final Pose2d RED_RIGHT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(657.37),
            Units.inchesToMeters(25.8),
            new Rotation2d(Units.degreesToRadians(126)));

    public static final Pose2d BLUE_PROCESSOR =
        new Pose2d(
            Units.inchesToMeters(235.73),
            Units.inchesToMeters(0),
            new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d RED_PROCESSOR =
        new Pose2d(
            Units.inchesToMeters(455.15),
            Units.inchesToMeters(317),
            new Rotation2d(Units.degreesToRadians(270)));

    // We wont be able to pick these game pieces up
    public static final Pose2d BLUE_LEFT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(47.967),
            Units.inchesToMeters(86.50),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_MIDDLE_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(47.967),
            Units.inchesToMeters(158.50),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_RIGHT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(47.967),
            Units.inchesToMeters(230.50),
            new Rotation2d(Units.degreesToRadians(0)));

    public static final Pose2d RED_LEFT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(642.903),
            Units.inchesToMeters(230.50),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_MIDDLE_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(642.903),
            Units.inchesToMeters(158.50),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_RIGHT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(642.903),
            Units.inchesToMeters(86.50),
            new Rotation2d(Units.degreesToRadians(180)));
  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final double DEADBAND_VALUE = 0.05;
  }
}
