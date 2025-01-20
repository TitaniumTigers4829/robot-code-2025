package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

public final class Constants {

  public static final class LogPaths {
    public static final String SYSTEM_PERFORMANCE_PATH = "SystemPerformance/";
    public static final String PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/";
    public static final String APRIL_TAGS_VISION_PATH = "Vision/AprilTags/";
  }

  public static final RobotType CURRENT_MODE = RobotType.SIM_ROBOT;

  public static enum RobotType {
    /** Running on a real robot. */
    COMP_ROBOT,

    /** Running a physics simulator. */
    SIM_ROBOT,

    /** Replaying from a log file. */
    DEV_ROBOT,

    /** Running the swerve robot. */
    SWERVE_ROBOT
  }

  /**
   * This is where we place constants related to hardware on a robot that aren't specific to any
   * singular subsystem.
   */
  public static final class HardwareConstants {
    public static final double TIMEOUT_S = 0.02;

    public static final double STATUS_SIGNAL_FREQUENCY = 50;

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
