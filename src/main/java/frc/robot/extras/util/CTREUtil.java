// // Copyright 2021-2024 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot.extras.util;

// import static edu.wpi.first.units.Units.*;
// import static edu.wpi.first.units.Units.Seconds;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.CANcoderSimState;
// import com.ctre.phoenix6.sim.ChassisReference;
// import com.ctre.phoenix6.sim.TalonFXSimState;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.extras.sim.SimArena;
// import frc.robot.extras.sim.SimMotorController;
// import frc.robot.extras.sim.SimMechanism.MechanismOutputs;

// public final class CTREUtil {
//     /** Attempts to run the command until no error is produced. */
//     public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
//         for (int i = 0; i < maxAttempts; i++) {
//             var error = command.get();
//             if (error.isOK()) break;
//         }
//     }

//     public static class TalonFXMotorControllerSim implements SimMotorController {
//         private static int instances = 0;
//         public final int id;

//         private final TalonFXSimState talonFXSimState;

//         public TalonFXMotorControllerSim(TalonFX talonFX, boolean motorInverted) {
//             this.id = instances++;

//             this.talonFXSimState = talonFX.getSimState();
//             talonFXSimState.Orientation =
//                     motorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
//         }

//         @Override
//         // @Override
//         public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
//             talonFXSimState.setSupplyVoltage(supply);
//             talonFXSimState.setRotorVelocity(state.velocity());
//             return null;
//         }
//         // @Override
//         // public Voltage updateControlSignal(
//         //         Angle mechanismAngle,
//         //         AngularVelocity mechanismVelocity,
//         //         Angle encoderAngle,
//         //         AngularVelocity encoderVelocity) {
//         //     talonFXSimState.setRawRotorPosition(encoderAngle);
//         //     talonFXSimState.setRotorVelocity(encoderVelocity);
//         //     talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
//         //     return talonFXSimState.getMotorVoltageMeasure();
//         // }

//         @Override
//         public boolean brakeEnabled() {
//             // TODO Auto-generated method stub
//             throw new UnsupportedOperationException("Unimplemented method 'brakeEnabled'");
//         }
//     }

//     public static class TalonFXMotorControllerWithRemoteCancoderSim extends TalonFXMotorControllerSim {
//         private final CANcoderSimState remoteCancoderSimState;
//         private final Angle encoderOffset;

//         public TalonFXMotorControllerWithRemoteCancoderSim(
//                 TalonFX talonFX,
//                 boolean motorInverted,
//                 CANcoder cancoder,
//                 boolean encoderInverted,
//                 Angle encoderOffset) {
//             super(talonFX, motorInverted);
//             this.remoteCancoderSimState = cancoder.getSimState();
//             this.remoteCancoderSimState.Orientation =
//                     encoderInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
//             this.encoderOffset = encoderOffset;
//         }

//     //     @Override
//     //     public Voltage updateControlSignal(
//     //             Angle mechanismAngle,
//     //             AngularVelocity mechanismVelocity,
//     //             Angle encoderAngle,
//     //             AngularVelocity encoderVelocity) {
//     //         remoteCancoderSimState.setRawPosition(mechanismAngle.minus(encoderOffset));
//     //         remoteCancoderSimState.setVelocity(mechanismVelocity);

//     //         return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
//     //     }
//     // }
//     }
//     public static double[] getSimulationOdometryTimeStamps() {
//         final double[] odometryTimeStamps = new double[SimArena.SimEnvTiming()];
//         for (int i = 0; i < odometryTimeStamps.length; i++) {
//             odometryTimeStamps[i] = Timer.getFPGATimestamp()
//                     - 0.02
//                     + i * SimArena.getSimulationDt().in(Seconds);
//         }

//         return odometryTimeStamps;
//     }
// }