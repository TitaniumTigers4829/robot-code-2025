package frc.robot.subsystems.funnyPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalFunnelPivot implements FunnelPivotInterface {
  private final TalonFX funnelMotor;
  private final CANcoder funnelEncoder;
  private final TalonFXConfiguration funnelMotorConfig;
  private final CANcoderConfiguration funnelEncoderConfig;
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
    funnelEncoder = new CANcoder(FunnelConstants.FUNNEL_ENCODER_MOTOR_ID);
    funnelMotorConfig = new TalonFXConfiguration();
    funnelEncoderConfig = new CANcoderConfiguration();
    mmPositionRequest = new MotionMagicVoltage(0);

    funnelVoltage = funnelMotor.getMotorVoltage();
    funnelVelocity = funnelMotor.getVelocity();
    funnelAngle = funnelEncoder.getAbsolutePosition();
    funnelSupplyCurrent = funnelMotor.getSupplyCurrent();
    funnelStatorCurrent = funnelMotor.getStatorCurrent();
    voltageOut = new VoltageOut(0);

    funnelEncoderConfig.MagnetSensor.MagnetOffset = -FunnelConstants.ANGLE_ZERO;
    funnelEncoderConfig.MagnetSensor.SensorDirection = FunnelConstants.FUNNEL_ENCODER_REVERSED;
    funnelEncoder.getConfigurator().apply(funnelEncoderConfig, HardwareConstants.TIMEOUT_S);

    funnelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    funnelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    funnelMotorConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    funnelMotorConfig.Slot0.kP = FunnelConstants.PIVOT_P;
    funnelMotorConfig.Slot0.kI = FunnelConstants.PIVOT_I;
    funnelMotorConfig.Slot0.kD = FunnelConstants.PIVOT_D;
    funnelMotorConfig.Slot0.kG = FunnelConstants.PIVOT_G;
    funnelMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    funnelMotorConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
    funnelMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND;

    funnelMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    funnelMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    funnelMotorConfig.Feedback.FeedbackRemoteSensorID = funnelEncoder.getDeviceID();

    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FunnelConstants.MAX_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FunnelConstants.MIN_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        funnelAngle,
        funnelVelocity,
        funnelVoltage,
        funnelSupplyCurrent,
        funnelStatorCurrent);
  }

  @Override
  public void updateInputs(AlgaePivotInputs inputs) {
    inputs.funnelAngle = funnelAngle.getValueAsDouble();
    inputs.funnelVelocity = funnelVelocity.getValueAsDouble();
    inputs.funnelVoltage = funnelVoltage.getValueAsDouble();
  }

  @Override
  public void setAlgaeSpeed(double speed) {
    funnelMotor.set(speed);
  }

  @Override
  public void setAlgaeAngle(double angle) {
    funnelTargetAngle = angle;
    funnelMotor.setControl(mmPositionRequest.withPosition(angle));
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    funnelMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public double getAlgaeAngle() {
    funnelAngle.refresh();
    return funnelAngle.getValueAsDouble();
  }

  @Override
  public double getAlgaePivotTarget() {
    return funnelTargetAngle;
  }
}
