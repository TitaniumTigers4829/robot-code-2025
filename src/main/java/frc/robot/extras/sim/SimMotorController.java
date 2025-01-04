package frc.robot.extras.sim;

import static edu.wpi.first.units.Units.Volts;

import sham.ShamMechanism.MechanismOutputs;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * An interface to represent a motor controller in the simulation.
 * This can be used to allow CTRE/REV sim to work with the sham library.
 */
public interface SimMotorController {
    /**
     * Runs the simulation for the motor.
     * 
     * @param dt the time step, this will not be different between invocations
     *     of this method on the same instance
     * @param supply the supply voltage
     * @param state the current state of the mechanism,
     *    this is the state <b>at the rotor</b>.
     *    If you wish to scale these states based on the gearbox ratio,
     *    you can do `state.times(gearboxRatio)`.
     * @return the output voltage
     */
    Voltage run(Time dt, Voltage supply, MechanismOutputs state);

    /**
     * Returns whether the brake is enabled.
     * 
     * @return whether the brake is enabled
     */
    boolean brakeEnabled();

    /**
     * Returns a motor controller that does nothing.
     * 
     * @return a motor controller that does nothing
     */
    public static SimMotorController none() {
        return new SimMotorController() {
            @Override
            public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
                return Volts.of(0);
            }

            @Override
            public boolean brakeEnabled() {
                return false;
            }
        };
    }
}
