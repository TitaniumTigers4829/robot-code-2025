package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriverController extends ControllerBase {

  public DriverController(int port, Subsystem... subsystems) {
    super(port, true);

    // disregard null safety for subsystems as it is checked on assignment

    /// FACE BUTTONS
    this.A.onTrue(Commands.none());

    this.B.onTrue(Commands.none());

    this.X.onTrue(Commands.none());

    this.Y.onTrue(Commands.none());

    /// BUMPER
    this.RB.onTrue(Commands.none());

    this.LB.onTrue(Commands.none());

    /// CENTER BUTTONS
    this.Back.onTrue(Commands.none());

    this.Start.onTrue(Commands.none());

    /// STICKS
    this.LS.onTrue(Commands.none());

    this.RS.onTrue(Commands.none());

    /// TRIGGERS
    this.LT.onTrue(Commands.none());

    this.RT.onTrue(Commands.none());

    /// DPAD
    this.DPR.onTrue(Commands.none());

    this.DPD.onTrue(Commands.none());

    this.DPL.onTrue(Commands.none());

    this.DPU.onTrue(Commands.none());
  }
}