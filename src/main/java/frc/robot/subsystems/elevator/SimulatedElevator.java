// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.elevator;

// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.elevator.ElevatorInterface.ElevatorInputs;

// public class SimulatedElevator implements ElevatorInterface {
//   private final ElevatorSim m_elevatorSim;
//   private final Encoder m_encoder;
//   private final PIDController m_pidController;
//   private final ElevatorFeedforward m_feedforward;
//   private double currentVolts;

//   public SimulatedElevator() {
//     m_elevatorSim = new ElevatorSim( //we dont know the
// design.............................................................
//         DCMotor.getVex775Pro(2),
//         10.0,
//         7.5,
//         0.75,
//         0.0,
//         2.056,
//         true, 0, null
//     );
//     m_encoder = new Encoder(0, 1);
//     m_pidController = new
// PIDController(ElevatorConstants.ELEVATOR_P,ElevatorConstants.ELEVATOR_I,ElevatorConstants.ELEVATOR_D);
//     m_feedforward = new
// ElevatorFeedforward(ElevatorConstants.ELEVATOR_S,ElevatorConstants.ELEVATOR_V,ElevatorConstants.ELEVATOR_G,ElevatorConstants.ELEVATOR_A);
//     currentVolts = 0.0;
//   }

//   public void updateInputs(ElevatorInputs inputs) {
//     m_elevatorSim.update(0.02);
//     inputs.leaderMotorPosition = m_encoder.getDistance();
//     inputs.followerMotorPosition = m_encoder.getDistance();
//   }

//   public double getElevatorPosition() {
//     return m_encoder.getDistance();
//   }

//   public void setElevatorPosition(double position) {
//     m_pidController.setSetpoint(position);
//     double output = m_pidController.calculate(m_encoder.getDistance());
//     double feedforward = m_feedforward.calculate(m_pidController.getSetpoint());
//     setVolts(output + feedforward);
//   }

//   @Override
//   public void setVolts(double volts){
//     double output = m_pidController.calculate(m_encoder.getDistance());
//     double feedforward = m_feedforward.calculate(m_pidController.getSetpoint());
//     currentVolts = output + feedforward;
//     m_elevatorSim.setInputVoltage(currentVolts);
//   }

//   public double getVolts() {
//     return currentVolts;
//   }
// }
