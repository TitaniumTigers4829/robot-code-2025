// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SimulatedFlywheel implements FlywheelInterface {
  private FlywheelSim simulatedFlywheel = new FlywheelSim(null, DCMotor.getFalcon500(2), 0);
  private PIDController simPID;
  private double currentVolts;

  public SimulatedFlywheel() {
    simPID =
        new PIDController(
            FlywheelConstants.FLYWHEEL_P,
            FlywheelConstants.FLYWHEEL_I,
            FlywheelConstants.FLYWHEEL_D);
  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.leaderMotorPosition = getFlywheelSpeed();
    inputs.followerMotorPosition = getFlywheelSpeed();
  }

  public void setFlywheelSpeed(double speed) {
    setVolts(simPID.calculate(getFlywheelSpeed(), speed));
  }

  public double getFlywheelSpeed() {
    return simulatedFlywheel.getAngularVelocityRPM();
  }

  public void setVolts(double volts) {
    currentVolts = simPID.calculate(volts);
    simulatedFlywheel.setInputVoltage(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
