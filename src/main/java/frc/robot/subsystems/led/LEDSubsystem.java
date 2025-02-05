package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private Spark spark;

  public LEDSubsystem() {
    spark = new Spark(0);
  }
}
