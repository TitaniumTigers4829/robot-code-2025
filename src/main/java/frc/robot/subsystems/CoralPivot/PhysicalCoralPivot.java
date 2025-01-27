package frc.robot.subsystems.CoralPivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.CoralPivot.CoralPivotInterface.coralPivotInputs;

public class PhysicalCoralPivot {
    private final TalonFX motor = new TalonFX(CoralConstants.coralPivotMotorID);
    private final CANcoder coralEncoder = new CANcoder(CoralConstants.coralEncoderID);
    private final TalonFXConfiguration coralMotorConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration coralEncoderConfig = new CANcoderConfiguration();
    private final StatusSignal<Voltage> coralVoltage = motor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> coralVelocity = motor.getVelocity();
    private final StatusSignal<Angle> coralAngle = motor.getPosition();
    private final StatusSignal<Current> coralSupplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Current> coralStatorCurrent = motor.getStatorCurrent();

    public PhysicalCoralPivot(){
        coralMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        coralMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        coralMotorConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

        coralEncoderConfig.MagnetSensor.SensorDirection = CoralConstants.coralEncoderReversed;
        
        }

    @Override
    public void updateInputs(coralPivotInputs inputs){
        inputs.coralAngle = coralAngle.getValueAsDouble();
        inputs.coralVelocity = coralVelocity.getValueAsDouble();
        inputs.coralVoltage = coralVoltage.getValueAsDouble();
    }
}
