// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbPivot extends SubsystemBase {
  /** Creates a new ClimbPivot. */
  public TalonFX climbPivotMotor = new TalonFX(PivotConstants.CLIMB_PIVOT_MOTOR_ID);

  private static final int TICKS_PER_ROTATION = 2048;
  private static final double GEAR_RATIO = 10.0;

  public ClimbPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    climbPivotMotor.getConfigurator().apply(config);
  }

  public void setPivotPosition(double angle) {
    climbPivotMotor.set(angle);
  }

  public double getCurrentPosition() {
    return climbPivotMotor.getPosition().getValueAsDouble();
  }

  public void stop(double endAngle) {
    climbPivotMotor.set(endAngle);
  }

  public void periodic() {}
}
