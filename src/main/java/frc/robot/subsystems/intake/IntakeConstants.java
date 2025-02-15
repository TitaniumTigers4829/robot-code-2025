package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int INTAKE_MOTOR_ID = 0;

  public static final int INTAKE_SPEED = 0 - 9;
  public static final int EJECT_SPEED = 0 - 9;

  // the max amount of stator and supply current allowed in the motor, respectively
  public static final double INTAKE_STATOR_LIMIT = 0.0;
  public static final double INTAKE_SUPPLY_LIMIT = 0.0;
  // Enables stator and supply current limits, respectively
  public static final boolean INTAKE_STATOR_LIMIT_ENABLE = true;
  public static final boolean INTAKE_SUPPLY_LIMIT_ENABLE = true;
}
