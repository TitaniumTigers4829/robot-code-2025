package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.ElevatorInterface.ElevatorInputs;

/** Add your docs here. */
public class PhysicalElevator {
  private TalonFX leaderMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
  private TalonFX followerMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);

  StatusSignal<Angle> leaderPosition;
  StatusSignal<Angle> followerPosition;

  StatusSignal<Voltage> leaderAppliedVoltage;
  StatusSignal<Voltage> followerAppliedVoltage;

  public PhysicalElevator() {
    leaderPosition = leaderMotor.getRotorPosition();
    followerPosition = followerMotor.getRotorPosition();

    leaderAppliedVoltage = leaderMotor.getMotorVoltage();
    followerAppliedVoltage = followerMotor.getMotorVoltage();

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.Slot0.kP = ElevatorConstants.ELEVATOR_P;
    elevatorConfig.Slot0.kI = ElevatorConstants.ELEVATOR_I;
    elevatorConfig.Slot0.kD = ElevatorConstants.ELEVATOR_D;

    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);
  }

  public void updateInputs(ElevatorInputs inputs) {
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
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
    ;
  }

  public double getVolts() {
    return leaderMotor.getMotorVoltage().getValueAsDouble();
  }
}
