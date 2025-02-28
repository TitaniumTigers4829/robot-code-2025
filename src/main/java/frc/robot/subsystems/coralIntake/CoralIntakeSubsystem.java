// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeSubsystem extends SubsystemBase {
  private CoralIntakeInterface coralIntakeInterface;
  private CoralIntakeInputsAutoLogged coralIntakeInputs = new CoralIntakeInputsAutoLogged();

  public CoralIntakeSubsystem(CoralIntakeInterface coralIntakeInterface) {
    this.coralIntakeInterface = coralIntakeInterface;
  }

  /**
   * Sets the intake to a desired speed
   *
   * @param speed the speed to set.
   */
  public void setIntakeSpeed(double speed) {
    coralIntakeInterface.setIntakeSpeed(speed);
  }

  public void gripCoral(double grippiness) {
    if (hasCoral()) {
      coralIntakeInterface.setIntakeVoltage(grippiness);
    }
  }

  /**
   * Checks if the coral intake has a coral or not
   *
   * @return true if the intake contains a coral game piece.
   */
  public boolean hasCoral() {
    return coralIntakeInputs.hasCoral || checkHasCoralWhenSensorCooked();
  }

  public boolean checkHasCoralWhenSensorCooked() {
    if ((coralIntakeInputs.intakeStatorCurrentAmps) > CoralIntakeConstants.INTAKE_STATOR_LIMIT) {
      return true;
    } else {
      return false;
    }
  }

  public void intakeCoral(double speed) {
    if (!hasCoral()) {
      setIntakeSpeed(speed);
    } else {
      setIntakeSpeed(0.0);
    }
  }

  @Override
  public void periodic() {
    coralIntakeInterface.updateInputs(coralIntakeInputs);
    Logger.processInputs("CoralIntakeSubsystem/", coralIntakeInputs);
  }

  public Command intakeCoral() {
    if (!this.hasCoral()) {
      return new StartEndCommand(
          // sets speed while command is active
          () -> this.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED),
          // sets speed when command ends
          () -> this.setIntakeSpeed(0),
          // requirements for command
          this);
    } else {
      return new StartEndCommand(
          // sets speed while command is active
          () -> this.setIntakeSpeed(0.0),
          // sets speed when command ends
          () -> this.setIntakeSpeed(0),
          // requirements for command
          this);
    }
  }

  public Command ejectCoral() {
    return new StartEndCommand(
        // sets speed while command is active
        () -> this.setIntakeSpeed(CoralIntakeConstants.EJECT_SPEED),
        // sets speed when command ends
        () -> this.setIntakeSpeed(0),
        // requirements for command
        this);
  }
}
