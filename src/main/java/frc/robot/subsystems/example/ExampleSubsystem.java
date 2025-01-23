// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.example;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  // this is the interface for the subsystem
  // code for motors and such would go here

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void exampleSubsystemFunction() {}

  public void setExamplePivotAngle(double angle) {
    // normal subsystem stuff
  }
  public double getExamplePivotAngle() {
    return 0.0;
  }
 
 
  public Command exampleFunctionalCommand() {
    // the command is written here instead of in it's own seperate file.
    // this saves space and makes the code more readable
    // it's only really good for commands that use 1 subsystem though(manual override commands and
    // such)
    return new FunctionalCommand(
        // does on init
        () -> this.setExamplePivotAngle(0),
        // does on execute
        () -> this.setExamplePivotAngle(1),
        // does when command ends
        interrupted -> this.setExamplePivotAngle(0), 
        // ends the command when this is true
        () -> getExamplePivotAngle() >= 0, 
        // requirements for the command
        this);
    
    
  }

  public Command exampleStartEndCommand() {
    // the command is written here instead of in it's own seperate file.
    // this saves space and makes the code more readable
    // it's only really good for commands that use 1 subsystem though(manual override commands and
    // such)
    return new StartEndCommand(
        // Sets the pivot to 1 while the command is active(runs on init())
        () -> this.setExamplePivotAngle(1), 
        // Sets the pivot to 0 when the command ends(runs on end())
        () -> this.setExamplePivotAngle(0), 
        // requirements for the command
        this);
  }

  public Command exampleRunOnceCommand() {
    // the command is written here instead of in it's own seperate file.
    // this saves space and makes the code more readable
    // it's only really good for commands that use 1 subsystem though(manual override commands and
    // such)

    // runs this one time
    return this.runOnce(() -> getExamplePivotAngle());
  }
}
