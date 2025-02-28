// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeSubsystem extends SubsystemBase {
  private CoralIntakeInterface coralIntakeInterface;
  private CoralIntakeInputsAutoLogged coralIntakeInputs = new CoralIntakeInputsAutoLogged();

  // States for the intake process
  private enum IntakeState {
    IDLE, // Chilling, not doing anything
    WAITING, // Waiting for coral to show up
    INGESTING, // Pulling the coral in
    REVERSING, // Backing up to position it
    STOPPED // All done, motor off
  }

  private IntakeState currentState = IntakeState.IDLE;
  private boolean previousSensorState = false;

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

  /**
   * Checks if the coral intake has a coral or not
   *
   * @return true if the intake contains a coral game piece.
   */
  public boolean hasCoral() {
    return coralIntakeInputs.isSensorConnected && coralIntakeInputs.hasCoral;
  }

  public Trigger getHasCoralTrigger() {
    return new Trigger(() -> hasCoral());
  }

  public void intakeCoral(double speed) {
    if (currentState == IntakeState.IDLE || currentState == IntakeState.STOPPED) {
      currentState = IntakeState.WAITING;
      previousSensorState = hasCoral(); // Set the starting sensor state
    }
    // if (coralIntakeInputs.isSensorConnected && !hasCoral()) {
    //   DoublePressTracker.doublePress(getHasCoralTrigger());
    //   setIntakeSpeed(speed);
    // } else {
    //   setIntakeSpeed(0.0);
    // }
  }

  // Tells you if the intake is done
  public boolean isIntakeComplete() {
    return currentState == IntakeState.STOPPED;
  }

  @Override
  public void periodic() {
    coralIntakeInterface.updateInputs(coralIntakeInputs);
    Logger.processInputs("CoralIntakeSubsystem/", coralIntakeInputs);

    // Check if coral is detected
    boolean currentSensorState = coralIntakeInputs.hasCoral;

    // Run the state machine
    switch (currentState) {
      case WAITING:
        if (!previousSensorState && currentSensorState) {
          // Coral just got detected: start pulling it in
          coralIntakeInterface.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED);
          currentState = IntakeState.INGESTING;
        }
        break;

      case INGESTING:
        if (previousSensorState && !currentSensorState) {
          // Coral’s gone from the sensor: reverse to position it
          coralIntakeInterface.setIntakeSpeed(-CoralIntakeConstants.INTAKE_SPEED);
          currentState = IntakeState.REVERSING;
        }
        break;

      case REVERSING:
        if (!previousSensorState && currentSensorState) {
          // Coral’s back in place: stop the motor
          coralIntakeInterface.setIntakeSpeed(0);
          currentState = IntakeState.STOPPED;
        }
        break;

      case STOPPED:
        // Chill until intakeCoral() is called again
        break;

      case IDLE:
        // Waiting for the signal to start
        break;
    }

    // Remember the sensor state for next time
    previousSensorState = currentSensorState;
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
