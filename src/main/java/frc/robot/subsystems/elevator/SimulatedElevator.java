// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimulatedElevator implements ElevatorInterface {
  private ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          ElevatorConstants.ELEVATOR_GEAR_RATIO,
          ElevatorConstants.ELEVATOR_CARRIAGE_MASS,
          ElevatorConstants.DRUM_RADIUS,
          ElevatorConstants.MIN_HEIGHT,
          ElevatorConstants.MAX_HEIGHT,
          true,
          0.0);
  private PIDController simPID;
  private double currentVolts;

  public SimulatedElevator() {
    simPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_P,
            ElevatorConstants.ELEVATOR_I,
            ElevatorConstants.ELEVATOR_D);
  }

  public void updateInputs(ElevatorInputs inputs) {
    inputs.leaderMotorPosition = getElevatorPosition();
    inputs.followerMotorPosition = getElevatorPosition();
  }

  public void setElevatorPosition(double position) {
    setVolts(simPID.calculate(getElevatorPosition(), position));
  }

  public double getElevatorPosition() {
    return elevatorSim.getPositionMeters();
  }

  public void setVolts(double volts) {
    currentVolts = simPID.calculate(volts);
    elevatorSim.setInputVoltage(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
