package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.HardwareConstants;

public class PhysicalCoralIntake implements CoralIntakeInterface {
  private final TalonFX motor = new TalonFX(CoralIntakeConstants.INTAKE_MOTOR_ID);
  private final StatusSignal<AngularVelocity> intakeVelocity = motor.getVelocity();
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public PhysicalCoralIntake() {
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    intakeConfig.CurrentLimits.StatorCurrentLimit = CoralIntakeConstants.INTAKE_STATOR_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_STATOR_LIMIT_ENABLE;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = CoralIntakeConstants.INTAKE_SUPPLY_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_SUPPLY_LIMIT_ENABLE;

    motor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void updateInputs(CoralIntakeInputs intakeInputs) {
    intakeInputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    intakeInputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public double getIntakeSpeed() {
    motor.getVelocity().refresh();
    return motor.getVelocity().getValueAsDouble();
  }
}
