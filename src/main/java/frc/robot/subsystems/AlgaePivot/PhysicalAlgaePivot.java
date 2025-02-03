package frc.robot.subsystems.AlgaePivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalAlgaePivot implements AlgaePivotInterface{
    private final TalonFX algaeMotor;
    private final CANcoder algaeEncoder;
    private final TalonFXConfiguration algaeMotorConfig;
    private final CANcoderConfiguration algaeEncoderConfig;
    private final StatusSignal<Voltage> algaeVoltage;
    private final StatusSignal<AngularVelocity> algaeVelocity;
    private StatusSignal<Angle> algaeAngle;
    private final StatusSignal<Current> algaeSupplyCurrent;
    private final StatusSignal<Current> algaeStatorCurrent;
    private final MotionMagicVoltage mmPositionRequest;
    private double algaeTargetAngle; 

    public PhysicalAlgaePivot(){
        algaeMotor = new TalonFX(AlgaeConstants.ALGAE_PIVOT_MOTOR_ID);
        algaeEncoder = new CANcoder(AlgaeConstants.ALGAE_ENCODER_MOTOR_ID);
        algaeMotorConfig = new TalonFXConfiguration();
        algaeEncoderConfig = new CANcoderConfiguration();
        mmPositionRequest = new MotionMagicVoltage(0);

        algaeVoltage = algaeMotor.getMotorVoltage();
        algaeVelocity = algaeMotor.getVelocity();
        algaeAngle = algaeEncoder.getAbsolutePosition();
        algaeSupplyCurrent = algaeMotor.getSupplyCurrent();
        algaeStatorCurrent = algaeMotor.getStatorCurrent();
        
        algaeEncoderConfig.MagnetSensor.MagnetOffset = -AlgaeConstants.ANGLE_ZERO;
        algaeEncoderConfig.MagnetSensor.SensorDirection = AlgaeConstants.ALGAE_ENCODER_REVERSED;
        algaeEncoder.getConfigurator().apply(algaeEncoderConfig, HardwareConstants.TIMEOUT_S);
        
        algaeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        algaeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        algaeMotorConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

        algaeMotorConfig.Slot0.kP = AlgaeConstants.PIVOT_P;
        algaeMotorConfig.Slot0.kI = AlgaeConstants.PIVOT_I;
        algaeMotorConfig.Slot0.kD = AlgaeConstants.PIVOT_D;
        algaeMotorConfig.Slot0.kG = AlgaeConstants.PIVOT_G;
        algaeMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        algaeMotorConfig.MotionMagic.MotionMagicAcceleration = 
        }

    @Override
    public void updateInputs(AlgaePivotInputs inputs){
        inputs.algaeAngle = algaeAngle.getValueAsDouble();
        inputs.algaeVelocity = algaeVelocity.getValueAsDouble();
        inputs.algaeVoltage = algaeVoltage.getValueAsDouble();
    }

    @Override
    public void setAlgaeSpeed(double speed){
        algaeMotor.set(speed);
    }

    @Override
    public void setAlgaeAngle(double angle){
        algaeTargetAngle = angle;
        algaeMotor.setControl(mmPositionRequest.withPosition(angle));
    }

    @Override
    public void setVoltage(double voltage){

    }

    @Override
    public double getAlgaeAngle(){
        algaeAngle.refresh();
        return algaeAngle.getValueAsDouble();
    }

    @Override
    public double getAlgaePivotTarget(){
        return algaeTargetAngle;
    }
}
