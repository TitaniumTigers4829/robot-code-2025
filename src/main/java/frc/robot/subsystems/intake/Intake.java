package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeInterface intakeInterface;
  private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  public Intake(IntakeInterface intakeInterface) {
    this.intakeInterface = intakeInterface;
  }

  /**
   * Sets the speed of the intake rollers
   *
   * @param speed intake roller speed (-1.0 to 1.0)
   */
  public void setIntakeSpeed(double speed) {
    intakeInterface.setIntakeSpeed(speed);
    Logger.recordOutput("OTBIntake/IntakeSpeed", speed);
  }

  @Override
  public void periodic() {
    intakeInterface.updateInputs(inputs);
    Logger.processInputs("OTBIntake/", inputs);
  }
}
