package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulatedIntake implements IntakeInterface {
  DCMotorSim intakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.01, 1),
          DCMotor.getFalcon500(1),
          0);
  private double intakeAppliedVolts = 0.0;

  public SimulatedIntake() {}

  @Override
  public void updateInputs(IntakeInputs inputs) {
    intakeSim.update(0.02);

    inputs.intakeVelocity =
        RadiansPerSecond.of(intakeSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeSim.setInputVoltage(speed);
  }

  @Override
  public double getIntakeSpeed() {
    return RadiansPerSecond.of(intakeSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
  }
}
