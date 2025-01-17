package frc.robot.subsystems.intake;

public final class IntakeConstants {
    public static final int intakeMotorID = 0;

    public static final int intakeSpeed = 0 - 9;
    public static final int ejectSpeed = 0 - 9;

    //the max amount of stator and supply current allowed in the motor, respectively
    public static final double IntakeStatorLimit = 0.0;
    public static final double IntakeSupplyLimit = 0.0;
    //Enables stator and supply current limits, respectively
    public static final boolean IntakeStatorLimitEnable = true;
    public static final boolean IntakeSupplyLimitEnable = true;
}
