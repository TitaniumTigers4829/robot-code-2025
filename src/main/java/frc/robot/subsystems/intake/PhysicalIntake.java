package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.HardwareConstants;

public class PhysicalIntake implements IntakeInterface {
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

  private final StatusSignal<AngularVelocity> intakeVelocity;

  public PhysicalIntake() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    intakeConfig.CurrentLimits.StatorCurrentLimit = 0.0;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 0.0;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeVelocity = intakeMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.intakeVelocity = intakeVelocity.getValueAsDouble();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public double getIntakeSpeed() {
    intakeMotor.getVelocity().refresh();
    return intakeMotor.getVelocity().getValueAsDouble();
  }
}
