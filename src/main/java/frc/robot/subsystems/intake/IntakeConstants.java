package frc.robot.subsystems.intake;

// This cool file contains constants for our intake
public final class IntakeConstants {
  public static final int INTAKE_MOTOR_ID = 0;
  public static final int LEFT_INTAKE_PIVOT_MOTOR_ID = 62;
  public static final int RIGHT_INTAKE_PIVOT_MOTOR_ID = 0 - 9;

  public static final int NOTE_SENSOR_ID = 0 - 9;

  public static final double INTAKE_PIVOT_GEAR_RATIO = 8.0;

  public static final double INTAKE_PIVOT_OUT = 0 - 9;
  public static final double INTAKE_PIVOT_IN = 0 - 9;

  public static final double INTAKE_PIVOT_NEUTRAL_SPEED = 0.0;

  public static final double OUTTAKE_SPEED = 0 - 9;
  public static final double INTAKE_SPEED = 0.8;
  public static final double INTAKE_NEUTRAL_SPEED = 0.0;
  public static final double FLAPPER_SPEED = 1.0;

  public static final double INTAKE_STATOR_LIMIT = 60;
  public static final double INTAKE_SUPPLY_LIMIT = 40;
  public static final boolean INTAKE_STATOR_ENABLE = true;
  public static final boolean INTAKE_SUPPLY_ENABLE = true;
}
