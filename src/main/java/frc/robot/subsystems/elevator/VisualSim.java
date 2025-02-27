// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class VisualSim {
    public Mechanism2d visualSim = new Mechanism2d(ElevatorConstants.ELEVATOR_WIDTH, ElevatorConstants.ELEVATOR_WHOLE_THING_ACTUAL_HEIGHT);
    
    public MechanismRoot2d root = visualSim.getRoot("Elevator root", 0, 0);
    
    private final MechanismLigament2d m_elevator;
    
    private Encoder m_elevatorEncoder;


    private static final double kElevatorMinimumLength = 0.0;

    public VisualSim() {
        m_elevator = new MechanismLigament2d(null, kElevatorMinimumLength, kElevatorMinimumLength);
        
        root.append(m_elevator);

        SmartDashboard.putData("Elevator2d", visualSim);
    }

   
    public void update(double length) {
        m_elevator.setLength(kElevatorMinimumLength + length);
 }
}