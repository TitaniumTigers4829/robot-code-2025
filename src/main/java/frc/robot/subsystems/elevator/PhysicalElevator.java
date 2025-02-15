// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalElevator implements ElevatorInterface {
  private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
  private final TalonFX followerMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  // private final Follower followerRequest =
  // new Follower(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID, true);

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Double> followerDutyCycle;
  private final StatusSignal<Double> leaderDutyCycle;

  private double desiredPosition;

  /** Creates a new PhysicalElevator. */
  public PhysicalElevator() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.Slot0.kP = ElevatorConstants.ELEVATOR_P;
    elevatorConfig.Slot0.kI = ElevatorConstants.ELEVATOR_I;
    elevatorConfig.Slot0.kD = ElevatorConstants.ELEVATOR_D;
    elevatorConfig.Slot0.kS = ElevatorConstants.ELEVATOR_S;
    elevatorConfig.Slot0.kV = ElevatorConstants.ELEVATOR_V;
    elevatorConfig.Slot0.kA = ElevatorConstants.ELEVATOR_A;
    elevatorConfig.Slot0.kG = ElevatorConstants.ELEVATOR_G;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Limits lol
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.REVERSE_LIMIT;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        ElevatorConstants.REVRESE_LIMIT_ENABLE;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.LIMIT;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ElevatorConstants.LIMIT_ENABLE;

    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable =
        ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;

    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configuration
    elevatorConfig.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.MOTION_MAGIC_MAX_ACCELERATION;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;

    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEAR_RATIO;

    // apply configuration

    leaderMotor.getConfigurator().apply(elevatorConfig);

    // followerMotor.setControl(followerRequest);
    followerMotor.getConfigurator().apply(elevatorConfig);

    // make the follower

    // Get info
    leaderPosition = leaderMotor.getPosition();
    leaderAppliedVoltage = leaderMotor.getMotorVoltage();
    followerPosition = followerMotor.getPosition();
    followerAppliedVoltage = followerMotor.getMotorVoltage();
    followerDutyCycle = followerMotor.getDutyCycle();
    leaderDutyCycle = leaderMotor.getDutyCycle();

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.STATUS_SIGNAL_FREQUENCY,
        leaderPosition,
        leaderAppliedVoltage,
        followerPosition,
        followerAppliedVoltage,
        leaderDutyCycle,
        followerDutyCycle);

    desiredPosition = 0.0;
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        followerPosition,
        leaderAppliedVoltage,
        followerAppliedVoltage,
        followerDutyCycle,
        leaderDutyCycle);
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.leaderMotorVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderDutyCycle = leaderDutyCycle.getValueAsDouble();
    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
    inputs.followerMotorVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerDutyCycle = followerDutyCycle.getValueAsDouble();
    inputs.desiredPosition = desiredPosition;
  }

  @Override
  public double getElevatorPosition() {
    leaderPosition.refresh();
    followerPosition.refresh();
    return leaderPosition.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double position) {
    desiredPosition = 1;
    // leaderMotor.setControl(mmPositionRequest.withPosition(position));
    // followerMotor.setControl(mmPositionRequest.withPosition(position));
    leaderMotor.set(-position);
    followerMotor.set(position);
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
