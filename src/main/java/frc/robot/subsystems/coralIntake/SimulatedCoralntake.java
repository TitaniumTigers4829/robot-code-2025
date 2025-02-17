package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulatedCoralntake implements CoralIntakeInterface {
  DCMotorSim simIntake =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.01, 1),
          DCMotor.getFalcon500(1));

  private double intakeAppliedVolts = 0.0;

  public SimulatedCoralntake() {}

  @Override
  public void updateInputs(CoralIntakeInputs intakeInputs) {
    simIntake.update(0.02);

    intakeInputs.intakeVelocity =
        RadiansPerSecond.of(simIntake.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
    intakeInputs.currentAmps = simIntake.getCurrentDrawAmps();
    intakeInputs.appliedVolts = intakeAppliedVolts;
  }

  @Override
  public double getIntakeSpeed() {
    return RadiansPerSecond.of(simIntake.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
  }
}
