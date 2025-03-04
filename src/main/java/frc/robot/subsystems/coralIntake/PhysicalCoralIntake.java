package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.leds.LEDConstants.LEDProcess;
import frc.robot.subsystems.leds.LEDSubsystem;

public class PhysicalCoralIntake implements CoralIntakeInterface {
  private final TalonFX coralIntakeMotor;
  private final DigitalInput coralSensor;
  private final LEDSubsystem ledSubsystem;

  private final Debouncer sensorDebouncer;

  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<Temperature> intakeTemperatureCelsius;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Double> intakeDutyCycle;
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public PhysicalCoralIntake() {
    ledSubsystem = new LEDSubsystem();
    coralIntakeMotor = new TalonFX(CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID);
    coralSensor = new DigitalInput(0);

    sensorDebouncer = new Debouncer(0.050, DebounceType.kRising);

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    intakeConfig.CurrentLimits.StatorCurrentLimit = CoralIntakeConstants.INTAKE_STATOR_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_STATOR_LIMIT_ENABLE;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = CoralIntakeConstants.INTAKE_SUPPLY_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_SUPPLY_LIMIT_ENABLE;

    coralIntakeMotor.getConfigurator().apply(intakeConfig);

    intakeVelocity = coralIntakeMotor.getVelocity();
    intakePosition = coralIntakeMotor.getPosition();
    intakeStatorCurrent = coralIntakeMotor.getStatorCurrent();
    intakeSupplyCurrent = coralIntakeMotor.getSupplyCurrent();
    intakeTemperatureCelsius = coralIntakeMotor.getDeviceTemp();
    intakeAppliedVolts = coralIntakeMotor.getMotorVoltage();
    intakeDutyCycle = coralIntakeMotor.getDutyCycle();
    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.RIO_SIGNAL_FREQUENCY,
        intakeVelocity,
        intakePosition,
        intakeStatorCurrent,
        intakeStatorCurrent,
        intakeSupplyCurrent,
        intakeTemperatureCelsius,
        intakeTemperatureCelsius,
        intakeAppliedVolts,
        intakeDutyCycle);
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
            intakeAppliedVolts,
            intakeDutyCycle);
    intakeInputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    intakeInputs.intakeStatorCurrentAmps = intakeStatorCurrent.getValueAsDouble();
    intakeInputs.intakeTemp = intakeTemperatureCelsius.getValueAsDouble();
    intakeInputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    intakeInputs.intakePosition = intakePosition.getValueAsDouble();
    intakeInputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
    intakeInputs.hasCoral =
        sensorDebouncer.calculate(!coralSensor.get()); // true if coral is sensed
    intakeInputs.intakeDutyCycle = intakeDutyCycle.getValueAsDouble();
    if (intakeInputs.hasCoral) {
        ledSubsystem.setProcess(LEDProcess.GREEN);
    }
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

  @Override
  public void setIntakeVoltage(double volts) {
    coralIntakeMotor.setControl(voltageOut.withOutput(volts));
  }
}
