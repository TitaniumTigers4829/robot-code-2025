// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * This 
 */
public class ClimbPivot extends SubsystemBase {
  /** Creates a new ClimbPivot. */
  public TalonFX climbPivotMotor = new TalonFX(PivotConstants.CLIMB_PIVOT_MOTOR_ID);

  public ClimbPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    climbPivotMotor.getConfigurator().apply(config);
  }

  /**
   * angle is in rotations,        idk where the angles are or what 0 is
   */
  public void setClimbPivotPosition(double angle) {
    climbPivotMotor.setPosition(angle);
  }

  /**
   * in rotations, i think that's a problem y'all
  */
  public double getClimbPivotPosition() {
    return climbPivotMotor.getPosition().getValueAsDouble();
  }
/**
 * in rotations, endAngle is something i guess i need help
 * 
 * @param endAngle this is the angle where the pivot ends
 */
  public void stop(double endAngle) {
    climbPivotMotor.setPosition(endAngle);
  }

@Override
  public void periodic() {}

  public Command setAngle(double targetPosition) {
    return new StartEndCommand(
            () -> setClimbPivotPosition(targetPosition),
            () -> stop(targetPosition),
            this
        ).until(() -> isPivotCloseEnough(targetPosition));
}

// Private helper method to check if the pivot is close enough to the target position
private boolean isPivotCloseEnough(double targetPosition) {
    return Math.abs(getClimbPivotPosition() - targetPosition) < PivotConstants.PIVOT_ACCEPTABLE_ERROR_DEGREES;
}
}
