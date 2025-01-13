package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulatedIntake implements IntakeInterface{
    DCMotorSim simIntake = new DCMotorSim(LinearSystemId.createDCMotorSystem(
        DCMotor.getFalcon500(1), 0.01, 1), 
        DCMotor.getFalcon500(1), 0);
    
    private double intakeAppliedVolts = 0.0;

    public SimulatedIntake(){}

    @Override
    public void updateInputs(IntakeInputs intakeInputs){
        simIntake.update(0.02);

        intakeInputs.intakeVelocity = RadiansPerSecond.of(simIntake.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
        intakeInputs.currentAmps = simIntake.getCurrentDrawAmps();
        intakeInputs.appliedVolts = intakeAppliedVolts;
    }

    @Override
    public double getIntakeSpeed(){
        return RadiansPerSecond.of(simIntake.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
    }
}
