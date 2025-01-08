// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.elevator.ElevatorInterface.ElevatorInputs;

public class PhysicalElevator implements ElevatorInterface {

  /** Creates a new ElevatorHardware. */
  private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
  private final TalonFX followerMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);
  private final Follower followerMotorControl = new Follower(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID, true);

  StatusSignal<Angle> leaderPosition;
  StatusSignal<Angle> followerPosition;
  StatusSignal<Voltage> leaderAppliedVoltage;
  StatusSignal<Voltage> followerAppliedVoltage;

  public PhysicalElevator() {
    leaderPosition = leaderMotor.getRotorPosition();
    leaderAppliedVoltage = leaderMotor.getMotorVoltage();
    followerPosition = followerMotor.getRotorPosition();
    followerAppliedVoltage = followerMotor.getMotorVoltage();

    followerMotor.setControl(followerMotorControl);

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    
    elevatorConfig.Slot0.kP = ElevatorConstants.ELEVATOR_P;
    elevatorConfig.Slot0.kI = ElevatorConstants.ELEVATOR_I;
    elevatorConfig.Slot0.kD = ElevatorConstants.ELEVATOR_D;
    elevatorConfig.Slot0.kS = ElevatorConstants.ELEVATOR_S;
    elevatorConfig.Slot0.kV = ElevatorConstants.ELEVATOR_V;
    elevatorConfig.Slot0.kA = ElevatorConstants.ELEVATOR_A;

    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;

    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, leaderPosition, leaderAppliedVoltage, followerPosition, followerAppliedVoltage);
  }
  public void updateInputs(ElevatorInputs inputs) {
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.leaderMotorVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
    inputs.followerMotorVoltage = followerAppliedVoltage.getValueAsDouble();
  }

  public double getElevatorPosition() {
    return leaderPosition.getValueAsDouble();
  }

  public void setElevatorPosition(double position) {
    leaderMotor.setPosition(position);
    followerMotor.setPosition(position);
  }

  public void setVolts(double volts) {
    leaderMotor.setVoltage(volts);
    followerMotor.setVoltage(volts);
  }

  public double getVolts() {
    return leaderMotor.getMotorVoltage().getValueAsDouble();
  }

}
