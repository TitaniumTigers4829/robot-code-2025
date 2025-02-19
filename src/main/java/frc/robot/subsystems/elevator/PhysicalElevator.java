// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalElevator implements ElevatorInterface {
  private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
  private final TalonFX followerMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);
  // private final Follower follower;

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Double> followerDutyCycle;
  private final StatusSignal<Double> leaderDutyCycle;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Double> elevatorReference;

  private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  /** Creates a new PhysicalElevator. */
  public PhysicalElevator() {
    // follower = new Follower(leaderMotor.getDeviceID(), true);
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Limits
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

    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // configuration
    elevatorConfig.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.MOTION_MAGIC_MAX_ACCELERATION;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;

    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEAR_RATIO;

    leaderMotor.getConfigurator().apply(elevatorConfig);
    // followerMotor.setControl(follower);
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerMotor.getConfigurator().apply(elevatorConfig);

    leaderPosition = leaderMotor.getPosition();
    leaderAppliedVoltage = leaderMotor.getMotorVoltage();
    followerPosition = followerMotor.getPosition();
    followerAppliedVoltage = followerMotor.getMotorVoltage();
    followerDutyCycle = followerMotor.getDutyCycle();
    leaderDutyCycle = leaderMotor.getDutyCycle();
    leaderStatorCurrent = leaderMotor.getStatorCurrent();
    followerStatorCurrent = followerMotor.getStatorCurrent();
    elevatorReference = leaderMotor.getClosedLoopReference();

    leaderMotor.setPosition(0.0);
    followerMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.RIO_SIGNAL_FREQUENCY,
        leaderPosition,
        leaderAppliedVoltage,
        followerPosition,
        followerAppliedVoltage,
        leaderDutyCycle,
        followerDutyCycle,
        leaderStatorCurrent,
        followerStatorCurrent,
        elevatorReference);
    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        followerPosition,
        leaderAppliedVoltage,
        followerAppliedVoltage,
        followerDutyCycle,
        leaderDutyCycle,
        leaderStatorCurrent,
        followerStatorCurrent,
        elevatorReference);
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.leaderMotorVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderDutyCycle = leaderDutyCycle.getValueAsDouble();
    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
    inputs.followerMotorVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerDutyCycle = followerDutyCycle.getValueAsDouble();
    inputs.desiredPosition = elevatorReference.getValueAsDouble();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValueAsDouble();
    inputs.followerStatorCurrent = followerStatorCurrent.getValueAsDouble();
  }

  @Override
  public double getElevatorPosition() {
    leaderPosition.refresh();
    followerPosition.refresh();
    return leaderPosition.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double position) {
    leaderMotor.setControl(mmPositionRequest.withPosition(position));
    followerMotor.setControl(mmPositionRequest.withPosition(position));
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

  @Override
  public void openLoop(double output) {
    leaderMotor.setControl(dutyCyleOut.withOutput(output));
    followerMotor.setControl(dutyCyleOut.withOutput(-output));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    elevatorConfig.Slot0.kP = kP;
    elevatorConfig.Slot0.kI = kI;
    elevatorConfig.Slot0.kD = kD;
    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    elevatorConfig.Slot0.kS = kS;
    elevatorConfig.Slot0.kV = kV;
    elevatorConfig.Slot0.kA = kA;
    elevatorConfig.Slot0.kG = kG;
    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);
  }
}
