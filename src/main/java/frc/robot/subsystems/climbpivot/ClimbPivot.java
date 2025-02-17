// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/** This */
public class ClimbPivot extends SubsystemBase {
  /** Creates a new ClimbPivot. */
  ClimbPivotInterface climbPivotInterface;

  ClimbPivotInputsAutoLogged inputsAutoLogged;
  private ClimbPivotInputsAutoLogged inputs = new ClimbPivotInputsAutoLogged();

  public ClimbPivot(ClimbPivotInterface climbPivotInterface) {
    this.climbPivotInterface = climbPivotInterface;
    inputsAutoLogged = new ClimbPivotInputsAutoLogged();
  }

  /** angle is in rotations, idk where the angles are or what 0 is */
  public void setClimbPivotPosition(double angle) {
    climbPivotInterface.setClimbPivotPosition(angle);
  }

  /** in rotations, i think that's a problem y'all */
  public double getClimbPivotPosition() {
    return climbPivotInterface.getClimbPivotPosition();
  }

  public double getVolts(){
    return climbPivotInterface.getVolts();
}
  /**
   * in rotations, endAngle is something i guess i need help
   *
   * @param endAngle this is the angle where the pivot ends
   */
  public void stop(double endAngle) {
    climbPivotInterface.setClimbPivotPosition(endAngle);
  }

  @Override
  public void periodic() {
    climbPivotInterface.updateInputs(inputs);
    Logger.processInputs("Climb Pivot/", inputs);
  }

  public Command setAngle() {
    return new StartEndCommand(
            () -> setClimbPivotPosition(PivotConstants.TARGET_POSITION),
            () -> stop(PivotConstants.TARGET_POSITION),
            this)
        .until(() -> isPivotCloseEnough());
  
  }
 public Command manualPivot(DoubleSupplier targetAngle) {
  return new StartEndCommand(
           () -> setClimbPivotPosition(targetAngle.getAsDouble()),
           () -> stop(PivotConstants.TARGET_POSITION),
          this);
 }

  // Private helper method to check if the pivot is close enough to the target position
  private boolean isPivotCloseEnough() {
    return Math.abs(getClimbPivotPosition() - PivotConstants.TARGET_POSITION)
        < PivotConstants.PIVOT_ACCEPTABLE_ERROR_DEGREES;
  }

}
