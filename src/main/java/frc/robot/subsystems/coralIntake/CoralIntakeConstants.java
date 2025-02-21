package frc.robot.subsystems.coralIntake;

public final class CoralIntakeConstants {
  public static final int CORAL_INTAKE_MOTOR_ID = 30;

  public static final double INTAKE_SPEED = 0.5;
  public static final double EJECT_SPEED = 1.0;

  // the max amount of stator and supply current allowed in the motor, respectively
  public static final double INTAKE_STATOR_LIMIT = 80.0;
  public static final double INTAKE_SUPPLY_LIMIT = 0.0;
  // Enables stator and supply current limits, respectively
  public static final boolean INTAKE_STATOR_LIMIT_ENABLE = true;
  public static final boolean INTAKE_SUPPLY_LIMIT_ENABLE = false;
}
