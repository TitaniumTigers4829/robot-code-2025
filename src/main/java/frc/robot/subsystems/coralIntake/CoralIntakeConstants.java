package frc.robot.subsystems.coralIntake;

public final class CoralIntakeConstants {
  public static final int INTAKE_MOTOR_ID = 2;

  public static final int INTAKE_SPEED = 1;
  public static final int EJECT_SPEED = -1;

  // the max amount of stator and supply current allowed in the motor, respectively
  public static final double INTAKE_STATOR_LIMIT = 0.0;
  public static final double INTAKE_SUPPLY_LIMIT = 0.0;
  // Enables stator and supply current limits, respectively
  public static final boolean INTAKE_STATOR_LIMIT_ENABLE = false;
  public static final boolean INTAKE_SUPPLY_LIMIT_ENABLE = false;
}
