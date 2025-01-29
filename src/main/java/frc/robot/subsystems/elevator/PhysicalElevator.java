// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalElevator implements ElevatorInterface {

  /** Creates a new ElevatorHardware. */
  private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);

  private final TalonFX followerMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);

  StatusSignal<Angle> leaderPosition;
  StatusSignal<Angle> followerPosition;
  StatusSignal<Voltage> leaderAppliedVoltage;
  StatusSignal<Voltage> followerAppliedVoltage;

  public PhysicalElevator() {
    leaderPosition = leaderMotor.getRotorPosition();
    leaderAppliedVoltage = leaderMotor.getMotorVoltage();
    followerPosition = followerMotor.getRotorPosition();
    followerAppliedVoltage = followerMotor.getMotorVoltage();

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.Slot0.kP = ElevatorConstants.ELEVATOR_P;
    elevatorConfig.Slot0.kI = ElevatorConstants.ELEVATOR_I;
    elevatorConfig.Slot0.kD = ElevatorConstants.ELEVATOR_D;
    elevatorConfig.Slot0.kS = ElevatorConstants.ELEVATOR_S;
    elevatorConfig.Slot0.kV = ElevatorConstants.ELEVATOR_V;
    elevatorConfig.Slot0.kA = ElevatorConstants.ELEVATOR_A;
    elevatorConfig.Slot0.kG = ElevatorConstants.ELEVATOR_G;

    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable =
        ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;

    leaderMotor.getConfigurator().apply(elevatorConfig);

    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerMotor.getConfigurator().apply(elevatorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.STATUS_SIGNAL_FREQUENCY,
        leaderPosition,
        leaderAppliedVoltage,
        followerPosition,
        followerAppliedVoltage);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.leaderMotorVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
    inputs.followerMotorVoltage = followerAppliedVoltage.getValueAsDouble();
  }

  @Override
  public double getElevatorPosition() {
    return leaderPosition.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double position) {
    leaderMotor.setPosition(position);
    followerMotor.setPosition(position);
  }

  @Override
  public void setVolts(double volts) {
    leaderMotor.setVoltage(volts);
    followerMotor.setVoltage(volts);
  }

  @Override
  public double getVolts() {
    return leaderAppliedVoltage.getValueAsDouble();
  }
}
