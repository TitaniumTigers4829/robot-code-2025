package frc.robot.subsystems.funnelPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalFunnelPivot implements FunnelPivotInterface {
  private final TalonFX funnelMotor;
  private final CANcoder funnelEncoder; // Added encoder
  private final TalonFXConfiguration funnelMotorConfig;
  private final CANcoderConfiguration funnelEncoderConfig; // Added encoder config
  private final StatusSignal<Voltage> funnelVoltage;
  private final StatusSignal<AngularVelocity> funnelVelocity;
  private StatusSignal<Angle> funnelAngle;
  private final StatusSignal<Current> funnelSupplyCurrent;
  private final StatusSignal<Current> funnelStatorCurrent;
  private final MotionMagicVoltage mmPositionRequest;
  private double funnelTargetAngle;
  private final VoltageOut voltageOut;

  public PhysicalFunnelPivot() {
    funnelMotor = new TalonFX(FunnelConstants.FUNNEL_PIVOT_MOTOR_ID);
    funnelEncoder = new CANcoder(FunnelConstants.FUNNEL_ENCODER_ID); // Initialize encoder
    funnelMotorConfig = new TalonFXConfiguration();
    funnelEncoderConfig = new CANcoderConfiguration(); // Initialize encoder config
    mmPositionRequest = new MotionMagicVoltage(0);
    voltageOut = new VoltageOut(0);

    funnelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    funnelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    funnelMotorConfig.MotorOutput.DutyCycleNeutralDeadband =
        HardwareConstants.MIN_DUTY_CYCLE_DEADBAND;

    funnelMotorConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
    funnelMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND;

    funnelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    funnelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FunnelConstants.MAX_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FunnelConstants.MIN_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    funnelMotorConfig.Feedback.SensorToMechanismRatio = FunnelConstants.FUNNEL_GEAR_RATIO;

    funnelEncoderConfig.MagnetSensor.MagnetOffset = FunnelConstants.FUNNEL_ZERO_ANGLE; // Configure encoder zero point ezpz
    funnelEncoderConfig.MagnetSensor.SensorDirection = FunnelConstants.FUNNEL_ENCODER_DIRECTION; // Configure encoder direction

    funnelMotor.getConfigurator().apply(funnelMotorConfig);
    funnelEncoder.getConfigurator().apply(funnelEncoderConfig); // Apply encoder config

    funnelVoltage = funnelMotor.getMotorVoltage();
    funnelVelocity = funnelMotor.getVelocity();
    funnelAngle = funnelEncoder.getPosition(); // Use encoder for angle
    funnelSupplyCurrent = funnelMotor.getSupplyCurrent();
    funnelStatorCurrent = funnelMotor.getStatorCurrent();

    funnelMotor.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        funnelAngle,
        funnelVelocity,
        funnelVoltage,
        funnelSupplyCurrent,
        funnelStatorCurrent);

    funnelMotor.setPosition(FunnelConstants.ANGLE_ZERO);
    funnelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FunnelPivotInputs inputs) {
    BaseStatusSignal.refreshAll(funnelAngle);
    inputs.funnelAngle = funnelAngle.getValueAsDouble();
    inputs.funnelVelocity = funnelVelocity.getValueAsDouble();
    inputs.funnelVoltage = funnelVoltage.getValueAsDouble();
  }

  @Override
  public void setFunnelSpeed(double speed) {
    funnelMotor.set(speed);
  }

  @Override
  public void setFunnelAngle(double angle) {
    funnelTargetAngle = angle;
    funnelMotor.setControl(mmPositionRequest.withPosition(angle));
  }

  @Override
  public void setFunnelVoltage(double voltage) {
    funnelMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public double getFunnelAngle() {
    funnelAngle.refresh();
    return funnelAngle.getValueAsDouble();
  }

  @Override
  public double getFunnelPivotTarget() {
    return funnelTargetAngle;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {

    funnelMotorConfig.Slot0.kP = kP;
    funnelMotorConfig.Slot0.kI = kI;
    funnelMotorConfig.Slot0.kD = kD;

    funnelMotor.getConfigurator().apply(funnelMotorConfig);
  }
}
