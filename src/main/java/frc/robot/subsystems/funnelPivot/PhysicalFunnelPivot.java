package frc.robot.subsystems.funnelPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalFunnelPivot implements FunnelPivotInterface {
  private final TalonFX funnelMotor;
  private final TalonFXConfiguration funnelMotorConfig;
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
    funnelMotorConfig = new TalonFXConfiguration();
    mmPositionRequest = new MotionMagicVoltage(0);
    funnelVoltage = funnelMotor.getMotorVoltage();
    funnelVelocity = funnelMotor.getVelocity();
    funnelAngle = funnelMotor.getRotorPosition();
    funnelSupplyCurrent = funnelMotor.getSupplyCurrent();
    funnelStatorCurrent = funnelMotor.getStatorCurrent();
    voltageOut = new VoltageOut(0);

    funnelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    funnelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    funnelMotorConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    funnelMotorConfig.Slot0.kP = FunnelConstants.PIVOT_P;
    funnelMotorConfig.Slot0.kI = FunnelConstants.PIVOT_I;
    funnelMotorConfig.Slot0.kD = FunnelConstants.PIVOT_D;

    funnelMotorConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
    funnelMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND;

    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FunnelConstants.MAX_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FunnelConstants.MIN_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    funnelMotor.getConfigurator().apply(funnelMotorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        funnelAngle,
        funnelVelocity,
        funnelVoltage,
        funnelSupplyCurrent,
        funnelStatorCurrent);

    funnelMotor.setPosition(FunnelConstants.startingPos);
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
}
