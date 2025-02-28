// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class VisualSim {
  public LoggedMechanism2d visualSim =
      new LoggedMechanism2d(
          ElevatorConstants.ELEVATOR_WIDTH, ElevatorConstants.ELEVATOR_WHOLE_THING_ACTUAL_HEIGHT);

  public LoggedMechanismRoot2d root = visualSim.getRoot("Elevator root", 0, 0);

  private final LoggedMechanismLigament2d m_elevator;

  private static final double kElevatorMinimumLength = 0.0;

  public VisualSim() {
    // Provide a name for the ligament and set an appropriate initial angle (usually 90 degrees for
    // vertical motion)
    m_elevator =
        new LoggedMechanismLigament2d(
            "Elevator", kElevatorMinimumLength, 90.0, 5.0, new Color8Bit(Color.kPink));

    root.append(m_elevator);

    // Initialize the encoder if needed (assuming it's plugged into channels 0 and 1)

    Logger.recordOutput("Elevator2d", visualSim);
  }

  // Method to update the elevator visualization based on input length
  public void update(double length) {
    m_elevator.setLength(kElevatorMinimumLength + length);
  }
}
