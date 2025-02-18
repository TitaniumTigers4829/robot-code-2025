// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.climbpivot.ClimbPivotInterface.ClimbPivotInputs;

/** This */
public class ClimbPivot extends SubsystemBase {
  /** Creates a new ClimbPivot. */
  ClimbPivotInterface climbPivotInterface;
  
 
  public ClimbPivot(ClimbPivotInterface climbPivotInterface) {
    this.climbPivotInterface = climbPivotInterface;
  }

  /** angle is in rotations, idk where the angles are or what 0 is */
  public void setClimbPivotPosition(double angle) {
    climbPivotInterface.setClimbPivotPosition(angle);
  }

  /** in rotations, i think that's a problem y'all */
  public double getClimbPivotPosition() {
    return climbPivotInterface.getClimbPivotPosition();
  }

  public double getVolts() {
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
    // climbPivotInterface.updateInputs(LoggableInputs inputsAutoLogged);
    // Logger.processInputs("Climb Pivot/", LoggableInputs inputsAutoLogged);
  }

  public Command setAngle() {
    return new StartEndCommand(
            () -> setClimbPivotPosition(PivotConstants.TARGET_POSITION),
            () -> stop(PivotConstants.TARGET_POSITION),
            this)
        .until(() -> isPivotCloseEnough());
  }

//  public Command manualPivotClimb(XboxController controller, int pivotAxis) {
//         return new RunCommand(
//             () -> climbPivotInter(controller.getRawAxis(pivotAxis)), 
//             this
//         );
//  }

  // Private helper method to check if the pivot is close enough to the target position
  private boolean isPivotCloseEnough() {
    return Math.abs(getClimbPivotPosition() - PivotConstants.TARGET_POSITION)
        < PivotConstants.PIVOT_ACCEPTABLE_ERROR_DEGREES;
  }
}
