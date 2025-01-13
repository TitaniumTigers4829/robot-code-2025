// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeInterface intakeInterface;
  private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
 
  public IntakeSubsystem(IntakeInterface intakeInterface) {
    this.intakeInterface = intakeInterface;
  }

  public void setIntakeSpeed(double speed) {
    intakeInterface.setIntakeSpeed(speed);
  }
  @Override
  public void periodic(){
    intakeInterface.updateInputs(inputs);
    Logger.processInputs("IntakeSubsystem/", inputs);
  }
}
