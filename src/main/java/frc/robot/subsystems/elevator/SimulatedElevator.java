// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.HardwareConstants;
import frc.robot.sim.simMechanism.SlantedElevatorSim;

public class SimulatedElevator implements ElevatorInterface {
  private final SlantedElevatorSim elevatorSim;
  private final ProfiledPIDController pidController;
  private final ElevatorFeedforward feedForward;
  private final DIOSim limitSwitch;

  private double currentVolts;
  private double desiredPosition;

  public SimulatedElevator() {
    elevatorSim =
        new SlantedElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.ELEVATOR_GEAR_RATIO,
            ElevatorConstants.ELEVATOR_CARRIAGE_MASS,
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            ElevatorConstants.SIMULATE_GRAVITY,
            ElevatorConstants.INCLINE_ANGLE_RADIANS,
            ElevatorConstants.BASE_HEIGHT);
    pidController =
        new ProfiledPIDController(
            ElevatorConstants.ELEVATOR_P,
            ElevatorConstants.ELEVATOR_I,
            ElevatorConstants.ELEVATOR_D,
            ElevatorConstants.ELEVATOR_CONSTRAINTS);
    feedForward =
        new ElevatorFeedforward(
            ElevatorConstants.ELEVATOR_S,
            ElevatorConstants.ELEVATOR_G,
            ElevatorConstants.ELEVATOR_V,
            ElevatorConstants.ELEVATOR_A);
    limitSwitch = new DIOSim(ElevatorConstants.LIMIT_SWITCH_ID);
    limitSwitch.setInitialized(false);

    currentVolts = 0.0;
    desiredPosition = 0.0;
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    elevatorSim.update(HardwareConstants.TIMEOUT_S);

    inputs.leaderMotorPosition = elevatorSim.getPositionMeters();
    inputs.leaderMotorVoltage = currentVolts;
    inputs.followerMotorPosition = elevatorSim.getPositionMeters();
    inputs.followerMotorVoltage = currentVolts;
    inputs.limitSwitchActivated = isLimitSwitchActivated();
    inputs.desiredPosition = desiredPosition;
  }

  @Override
  public void setElevatorPosition(double position) {
    desiredPosition = position;
    pidController.setGoal(position);
    double output = pidController.calculate(getElevatorPosition());
    double feedforwardOutput = feedForward.calculate(pidController.getSetpoint().velocity);

    setVolts(output + feedforwardOutput);
  }

  @Override
  public double getElevatorPosition() {
    return elevatorSim.getPositionMeters();
  }

  @Override
  public void setVolts(double volts) {
    currentVolts = volts;
    elevatorSim.setInputVoltage(currentVolts);
  }

  @Override
  public double getVolts() {
    return currentVolts;
  }

  @Override
  public boolean isLimitSwitchActivated() {
    return !limitSwitch.getValue();
  }
}
