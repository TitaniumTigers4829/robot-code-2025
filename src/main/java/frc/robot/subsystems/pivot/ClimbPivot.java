// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbPivot extends SubsystemBase {
  /** Creates a new ClimbPivot. */
  public TalonFX climbPivotMotor = new TalonFX(Constants.CLIMB_PIVOT_MOTOR);
  private static final int TICKS_PER_ROTATION = 2048;
  private static final double GEAR_RATIO = 10.0;

  public ClimbPivot(TalonFX climbPivotMotor) {
    this.climbPivotMotor = climbPivotMotor;
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = 0.0;
    config.slot0.kI = 0.0;
    config.slot0.kD = 0.0;
    config.slot0.kF = 0.0;
    config.motionCruiseVelocity = 15000;
    climb.motionAcceleration = 6000;
    climbPivotMotor.configAllSettings(config);
    climbPivotMotor.setNeutralMode(NeutralMode.Brake);  
    climbPivotMotor.setSelectedSensorPosition(0);
  }

  public void setPivotPosition(double degrees) {
    double ticks = degreesToTicks(degrees);
    climbPivotMotor.set(ControlMode.MotionMagic, ticks);
}

public double getCurrentPosition() {
    return ticksToDegrees(climbPivotMotor.getSelectedSensorPosition());
}

public void stop() {
    climbPivotMotor.set(ControlMode.PercentOutput, 0);
}

private double degreesToTicks(double degrees) {
    return degrees / 360.0 * TICKS_PER_ROTATION * GEAR_RATIO;
}

private double ticksToDegrees(double ticks) {
    return ticks / (TICKS_PER_ROTATION * GEAR_RATIO) * 360.0;
}

@Override
public void periodic() {
    System.out.println("Current Position: " + getCurrentPosition());
}
}
