// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.extras.util;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.extras.sim.SimArena;
import frc.robot.extras.sim.SimMotorController;
import frc.robot.extras.sim.utils.GearRatio;
import frc.robot.extras.sim.SimMechanism.MechanismOutputs;

public final class CTREUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    public static class TalonFXSimController implements SimMotorController {
        private static int instances = 0;
        public final int id;

        private final AtomicBoolean brakeEnabled = new AtomicBoolean();

        private final TalonFXSimState talonFXSimState;



        public TalonFXSimController(TalonFXSimState talonFXSimState, InvertedValue motorInverted) {
            this.id = instances++;

            this.talonFXSimState = talonFXSimState;

            // jank af
            talonFXSimState.Orientation = motorInverted == InvertedValue.Clockwise_Positive ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        }

        @Override
        public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
            talonFXSimState.setSupplyVoltage(supply);
            talonFXSimState.setRotorVelocity(state.velocity());
            talonFXSimState.setRawRotorPosition(state.position());
            talonFXSimState.setRotorVelocity(state.velocity());
            talonFXSimState.setRotorAcceleration(state.acceleration());
            return talonFXSimState.getMotorVoltageMeasure();
        }

        public TalonFXSimController withBrakeEnabled(boolean brakeEnabled) {
            this.brakeEnabled.set(brakeEnabled);
            return this;
        }
    

        @Override
        public boolean brakeEnabled() {
            return brakeEnabled.get();
        }
    }

public class FusedTalonFxSimController implements SimMotorController {
    private final TalonFXSimState talonSimState;
    private final CANcoderSimState cancoderSimState;
    private final GearRatio rotorToSensor;
    private final AtomicBoolean brakeEnabled = new AtomicBoolean(false);

    public FusedTalonFxSimController(
        TalonFXSimState talonSimState,
        CANcoderSimState cancoderSimState,
        GearRatio rotorToSensor
    ) {
        this.talonSimState = talonSimState;
        this.cancoderSimState = cancoderSimState;
        this.rotorToSensor = rotorToSensor;
    }

    public FusedTalonFxSimController withBrakeEnabled(boolean brakeEnabled) {
        this.brakeEnabled.set(brakeEnabled);
        return this;
    }

    @Override
    public boolean brakeEnabled() {
        return brakeEnabled.get();
    }

    @Override
    public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
        talonSimState.setSupplyVoltage(supply);
        talonSimState.setRawRotorPosition(state.position());
        talonSimState.setRotorVelocity(state.velocity());
        talonSimState.setRotorAcceleration(state.acceleration());

        MechanismOutputs sensorState = state.times(rotorToSensor.getOverdrive());
        cancoderSimState.setSupplyVoltage(supply);
        cancoderSimState.setRawPosition(sensorState.position());
        cancoderSimState.setVelocity(sensorState.velocity());
        cancoderSimState.setMagnetHealth(MagnetHealthValue.Magnet_Green);

        return talonSimState.getMotorVoltageMeasure();
    }
}
}