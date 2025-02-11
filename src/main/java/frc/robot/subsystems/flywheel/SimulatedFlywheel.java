// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SimulatedFlywheel implements FlywheelInterface {
  private FlywheelSim simulatedFlywheel = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), FlywheelConstants.MOMENT_OF_INERTIA, FlywheelConstants.FLYWHEEL_GEAR_RATIO), DCMotor.getFalcon500(1));
  private double currentVolts;
  public TalonFXConfiguration config;

  private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  


  public SimulatedFlywheel() {
    config.Slot0.kP = FlywheelConstants.FLYWHEEL_P;
    config.Slot0.kI = FlywheelConstants.FLYWHEEL_I;
    config.Slot0.kD = FlywheelConstants.FLYWHEEL_D;

    config.Slot0.kS = FlywheelConstants.FF_FLYWHEEL_S;
    config.Slot0.kV = FlywheelConstants.FF_FLYWHEEL_V;
    config.Slot0.kA = FlywheelConstants.FF_FLYWHEEL_A;

  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelMotorSpeed = getFlywheelSpeed();
  }

  public void setFlywheelSpeed(double speed) {
    simulatedFlywheel.setControl(velocityRequest.withVelocity(speed));
  }

  public double getFlywheelSpeed(double speed){
    return simulatedFlywheel.getVelocity(velocityRequest.withVelocity(speed));
  }

  public void setVolts(double volts) {
    currentVolts = velocityRequest.calculate(volts);
    simulatedFlywheel.setInputVoltage(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
