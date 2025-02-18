package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.HardwareConstants;

public class PhysicalCoralIntake implements CoralIntakeInterface {
  private final TalonFX coralIntakeMotor;
  private final DigitalInput coralSensor;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<Temperature> intakeTemperatureCelsius;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public PhysicalCoralIntake() {
    coralIntakeMotor = new TalonFX(CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID);
    coralSensor = new DigitalInput(0);

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    intakeConfig.CurrentLimits.StatorCurrentLimit = CoralIntakeConstants.INTAKE_STATOR_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_STATOR_LIMIT_ENABLE;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = CoralIntakeConstants.INTAKE_SUPPLY_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_SUPPLY_LIMIT_ENABLE;

    coralIntakeMotor.getConfigurator().apply(intakeConfig);

    intakeVelocity = coralIntakeMotor.getVelocity();
    intakePosition = coralIntakeMotor.getPosition();
    intakeStatorCurrent = coralIntakeMotor.getStatorCurrent();
    intakeSupplyCurrent = coralIntakeMotor.getSupplyCurrent();
    intakeTemperatureCelsius = coralIntakeMotor.getDeviceTemp();
    intakeAppliedVolts = coralIntakeMotor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.RIO_SIGNAL_FREQUENCY,
        intakeVelocity,
        intakePosition,
        intakeStatorCurrent,
        intakeStatorCurrent,
        intakeSupplyCurrent,
        intakeTemperatureCelsius,
        intakeTemperatureCelsius,
        intakeAppliedVolts);
    coralIntakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralIntakeInputs intakeInputs) {
    BaseStatusSignal.refreshAll(intakeVelocity);
    intakeInputs.isConnected =
        BaseStatusSignal.isAllGood(
            intakeVelocity,
            intakeStatorCurrent,
            intakeSupplyCurrent,
            intakeTemperatureCelsius,
            intakePosition,
            intakeAppliedVolts);
    intakeInputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    intakeInputs.intakeStatorCurrentAmps = intakeStatorCurrent.getValueAsDouble();
    intakeInputs.intakeTemp = intakeTemperatureCelsius.getValueAsDouble();
    intakeInputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    intakeInputs.intakePosition = intakePosition.getValueAsDouble();
    intakeInputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
    intakeInputs.hasCoral = !coralSensor.get(); // true if coral is sensed
  }

  @Override
  public void setIntakeSpeed(double speed) {
    coralIntakeMotor.set(speed);
  }

  @Override
  public double getIntakeSpeed() {
    coralIntakeMotor.getVelocity().refresh();
    return coralIntakeMotor.getVelocity().getValueAsDouble();
  }
}
