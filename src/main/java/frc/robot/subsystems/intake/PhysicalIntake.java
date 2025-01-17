package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.HardwareConstants;

public class PhysicalIntake implements IntakeInterface{
    private final TalonFX motor = new TalonFX(IntakeConstants.intakeMotorID);
    private final StatusSignal<AngularVelocity> intakeVelocity = motor.getVelocity();
    private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    public PhysicalIntake(){
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

        intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.IntakeStatorLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.IntakeStatorLimitEnable;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.IntakeSupplyLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.IntakeSupplyLimitEnable;

        motor.getConfigurator().apply(intakeConfig);
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs){
        intakeInputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    }

    @Override
    public void setIntakeSpeed(double speed){
        motor.set(speed);
    }

    @Override
    public double getIntakeSpeed(){
        motor.getVelocity().refresh();
        return motor.getVelocity().getValueAsDouble();
    }
}
