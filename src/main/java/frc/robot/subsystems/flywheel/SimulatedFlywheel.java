// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SimulatedFlywheel implements FlywheelInterface {
  private FlywheelSim simulatedFlywheel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getFalcon500(1),
              FlywheelConstants.MOMENT_OF_INERTIA,
              FlywheelConstants.FLYWHEEL_GEAR_RATIO),
          DCMotor.getFalcon500(1));

  private double currentVolts;
  private PIDController simPID;
  private SimpleMotorFeedforward simFF;

  public SimulatedFlywheel() {
    simPID = new PIDController(FlywheelConstants.FLYWHEEL_P, FlywheelConstants.FLYWHEEL_I, FlywheelConstants.FLYWHEEL_D);
    simFF = new SimpleMotorFeedforward(FlywheelConstants.FF_FLYWHEEL_S, FlywheelConstants.FF_FLYWHEEL_V, FlywheelConstants.FF_FLYWHEEL_A);
  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelAppliedVolts = currentVolts;
    inputs.flywheelMotorSpeed = getFlywheelVelocity();
  }

  public void setFlywheelSpeed(double velocity) {
    setVolts(simPID.calculate(getFlywheelVelocity(), velocity) + simFF.calculate(velocity));
  }

  public double getFlywheelVelocity(double velocity) {
    return simulatedFlywheel.getAngularVelocity().in(RotationsPerSecond);
  }

  public void setVolts(double volts) {
    simulatedFlywheel.setInputVoltage(volts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
