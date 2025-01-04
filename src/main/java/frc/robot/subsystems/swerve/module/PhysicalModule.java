package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import java.util.Queue;

public class PhysicalModule implements ModuleInterface {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);

  private final Queue<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveMotorAppliedVoltage;
  private final StatusSignal<Current> driveMotorCurrent;

  private final Queue<Angle> turnEncoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> turnEncoderVelocity;
  private final StatusSignal<Voltage> turnMotorAppliedVolts;
  private final StatusSignal<Current> turnMotorCurrent;

  private final BaseStatusSignal[] periodicallyRefreshedSignals;

  public PhysicalModule(ModuleConfig moduleConfig) {
    driveMotor = new TalonFX(moduleConfig.driveMotorChannel(), DeviceCANBus.CANIVORE.name);
    turnMotor = new TalonFX(moduleConfig.turnMotorChannel(), DeviceCANBus.CANIVORE.name);
    turnEncoder = new CANcoder(moduleConfig.turnEncoderChannel(), DeviceCANBus.CANIVORE.name);

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
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
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

    drivePosition = OdometryThread.registerSignalInput(driveMotor.getPosition());
    driveVelocity = driveMotor.getVelocity();
    driveMotorAppliedVoltage = driveMotor.getMotorVoltage();
    driveMotorCurrent = driveMotor.getSupplyCurrent();

    turnEncoderAbsolutePosition =
        OdometryThread.registerSignalInput(turnEncoder.getAbsolutePosition());
    turnEncoderVelocity = turnEncoder.getVelocity();
    turnMotorAppliedVolts = turnMotor.getMotorVoltage();
    turnMotorCurrent = turnMotor.getSupplyCurrent();

    periodicallyRefreshedSignals =
        new BaseStatusSignal[] {
          driveVelocity,
          driveMotorAppliedVoltage,
          driveMotorCurrent,
          turnEncoderVelocity,
          turnMotorAppliedVolts,
          turnMotorCurrent
        };

    driveMotor.setPosition(0.0);
    turnMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, periodicallyRefreshedSignals);
    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.isConnected = BaseStatusSignal.isAllGood(periodicallyRefreshedSignals);

    inputs.driveVelocity = driveVelocity.getValueAsDouble();

    // Handle drive positions
    if (!drivePosition.isEmpty()) {
      Angle driveRelativePosition = Rotations.zero();
      for (Angle angle : drivePosition) {
        driveRelativePosition = angle;
      }
      inputs.drivePosition = driveRelativePosition.in(Rotations);
      drivePosition.clear();
    }

    // Handle turn absolute positions
    if (!turnEncoderAbsolutePosition.isEmpty()) {
      Rotation2d turnPosition = new Rotation2d();
      for (Angle angle : turnEncoderAbsolutePosition) {
        turnPosition = Rotation2d.fromRotations(angle.in(Rotations));
      }
      inputs.turnAbsolutePosition = turnPosition;
      turnEncoderAbsolutePosition.clear();
    }

    inputs.driveAppliedVolts = driveMotorAppliedVoltage.getValueAsDouble();
    inputs.driveCurrentAmps = driveMotorCurrent.getValueAsDouble();

    inputs.turnVelocity = turnEncoderVelocity.getValueAsDouble();
    inputs.turnAppliedVolts = turnMotorAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnMotorCurrent.getValueAsDouble();
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

    driveMotor.setControl(velocityRequest.withVelocity(RotationsPerSecond.of(desiredDriveRPS)));
    turnMotor.setControl(
        mmPositionRequest.withPosition(Rotations.of(desiredState.angle.getRotations())));
  }

  public double getTurnRotations() {
    turnEncoder.getAbsolutePosition().refresh();
    return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble())
        .getRotations();
  }

  @Override
  public void stopModule() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }
}
