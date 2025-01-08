package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class PhysicalIntake implements IntakeInterface{
    private final TalonFX motor = new TalonFX(0);
    private final StatusSignal<AngularVelocity> intakeSpeed;
    private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    public PhysicalIntake(){
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
    }
}
