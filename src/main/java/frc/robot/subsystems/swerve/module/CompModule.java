package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

public class CompModule implements ModuleInterface {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicTorqueCurrentFOC positionTorqueCurrentFOC =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveMotorAppliedVoltage;
  private final StatusSignal<Current> driveMotorCurrent;
  private final StatusSignal<Current> driveMotorTorque;
  private final StatusSignal<Double> driveMotorReference;

  private final StatusSignal<Angle> turnEncoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> turnEncoderVelocity;
  private final StatusSignal<Voltage> turnMotorAppliedVolts;
  private final StatusSignal<Current> turnMotorCurrent;
  private final StatusSignal<Current> turnMotorTorqueCurrent;
  private final StatusSignal<Double> turnMotorReference;

  public CompModule(ModuleConfig moduleConfig) {
    driveMotor =
        new TalonFX(moduleConfig.driveMotorChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnMotor =
        new TalonFX(moduleConfig.turnMotorChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnEncoder =
        new CANcoder(moduleConfig.turnEncoderChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -moduleConfig.angleZero();
    turnEncoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderReversed();
    turnEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
    driveConfig.Slot0.kA = ModuleConstants.DRIVE_A;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = moduleConfig.driveReversed();
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    // driveConfig/
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimit = ModuleConstants.DRIVE_STATOR_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveMotor.getConfigurator().apply(driveConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.Slot0.kP = ModuleConstants.TURN_P;
    turnConfig.Slot0.kI = ModuleConstants.TURN_I;
    turnConfig.Slot0.kD = ModuleConstants.TURN_D;
    turnConfig.Slot0.kS = ModuleConstants.TURN_S;
    turnConfig.Slot0.kV = ModuleConstants.TURN_V;
    turnConfig.Slot0.kA = ModuleConstants.TURN_A;
    turnConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.RotorToSensorRatio = 11;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = moduleConfig.turnReversed();
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity =
        ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotor.getConfigurator().apply(turnConfig, HardwareConstants.TIMEOUT_S);

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveMotorAppliedVoltage = driveMotor.getMotorVoltage();
    driveMotorCurrent = driveMotor.getSupplyCurrent();
    driveMotorReference = driveMotor.getClosedLoopReference();
    driveMotorTorque = driveMotor.getTorqueCurrent();

    turnEncoderAbsolutePosition = turnEncoder.getAbsolutePosition();
    turnEncoderVelocity = turnEncoder.getVelocity();
    turnMotorAppliedVolts = turnMotor.getMotorVoltage();
    turnMotorCurrent = turnMotor.getSupplyCurrent();
    turnMotorTorqueCurrent = turnMotor.getTorqueCurrent();
    turnMotorReference = turnMotor.getClosedLoopReference();

    driveMotor.setPosition(0.0);
    turnMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        drivePosition,
        turnEncoderAbsolutePosition,
        driveVelocity,
        driveMotorTorque,
        driveMotorReference,
        turnEncoderVelocity,
        turnMotorAppliedVolts,
        turnMotorCurrent,
        turnMotorReference,
        turnMotorTorqueCurrent);
    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    BaseStatusSignal.waitForAll(
        0.01,
        drivePosition,
        turnEncoderAbsolutePosition,
        driveVelocity,
        driveMotorTorque,
        driveMotorReference,
        turnEncoderVelocity,
        turnMotorAppliedVolts,
        turnMotorCurrent,
        turnMotorReference,
        turnMotorTorqueCurrent);

    inputs.isConnected =
        BaseStatusSignal.isAllGood(
            drivePosition,
            turnEncoderAbsolutePosition,
            driveVelocity,
            turnEncoderVelocity,
            driveMotorTorque,
            driveMotorReference,
            turnEncoderVelocity,
            turnMotorAppliedVolts,
            turnMotorCurrent,
            turnMotorReference,
            turnMotorTorqueCurrent);
    inputs.driveVelocity = driveVelocity.getValueAsDouble();
    inputs.drivePosition = -drivePosition.getValueAsDouble();
    inputs.driveDesiredPosition = driveMotorReference.getValueAsDouble();
    inputs.driveTorqueCurrent = driveMotorTorque.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnEncoderAbsolutePosition.getValueAsDouble());
    inputs.turnVelocity = turnEncoderVelocity.getValueAsDouble();
    inputs.turnDesiredPosition = turnMotorReference.getValueAsDouble();
    inputs.turnTorqueCurrent = turnMotorTorqueCurrent.getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    driveMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    turnMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    driveMotor.setControl(
        velocityTorqueCurrentFOC
            .withVelocity(RotationsPerSecond.of(desiredDriveRPS))
            .withUseTimesync(true));
    turnMotor.setControl(
        positionTorqueCurrentFOC
            .withPosition(Rotations.of(desiredState.angle.getRotations()))
            .withUseTimesync(true));
  }

  public double getTurnRotations() {
    turnEncoder.getAbsolutePosition().refresh();
    return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble())
        .getRotations();
  }

  @Override
  public double getDrivePositionRadians() {
    drivePosition.refresh();
    return 2.0 * Math.PI * (drivePosition.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO);
  }

  @Override
  public void stopModule() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }
}
