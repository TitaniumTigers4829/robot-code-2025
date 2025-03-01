package frc.robot.sim.simController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.sim.simController.SimMotorController.ControllerOutput;
import frc.robot.sim.simMechanism.SimMechanism.MechanismState;

public class CTRESimulation {
    private final UnitSafeMotorController customController;
    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSim;
    private final DCMotorSim motorSim;
    private double previousVelocity = 0.0;
    private final double dt = 0.02; // 20ms

    public CTRESimulation(UnitSafeMotorController controller, TalonFX fx, DCMotorSim sim) {
        this.customController = controller;
        this.talonFX = fx;
        this.talonFXSim = fx.getSimState();
        this.motorSim = sim;
        // Set supply voltage
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        // Configure motor model
        customController.configureMotorModel(createDCMotorExt()); // Assume this creates a matching motor model
    }

    private DCMotorExt createDCMotorExt() {
        // Implement based on motorSim's parameters, e.g., resistance, torque constant
        return new DCMotorExt() {
            @Override
            public int numMotors() { return 1; } // Example
            // Add other necessary methods like getResistance, getBackEMFConstant
        };
    }

    public void runSimulation() {
        while (true) {
            // Get current state from simulated motor
            Angle position = Angle.of(motorSim.getAngle());
            AngularVelocity velocity = AngularVelocity.of(motorSim.getAngularVelocity());
            AngularAcceleration acceleration = AngularAcceleration.of((motorSim.getAngularVelocity() - previousVelocity) / dt);
            previousVelocity = motorSim.getAngularVelocity();

            MechanismState state = new MechanismState() {
                @Override
                public Angle position() { return position; }
                @Override
                public AngularVelocity velocity() { return velocity; }
                @Override
                public AngularAcceleration acceleration() { return acceleration; }
            };

            // Run custom controller
            ControllerOutput output = customController.run(Time.of(Seconds.of(dt)), Voltage.of(RobotController.getBatteryVoltage()), state);

            // Handle output
            if (output instanceof ControllerOutput.VoltageOutput vo) {
                // Voltage output
                talonFX.setControl(output);
                double motorVoltage = talonFXSim.getMotorVoltage();
                motorSim.setInputVoltage(motorVoltage);
            } else if (output instanceof ControllerOutput.CurrentOutput co) {
                // Current output, approximate voltage
                Current desiredCurrent = co.current();
                double velocityRadPerSec = motorSim.getAngularVelocity().in(RadiansPerSecond);
                // Assume motor model provides resistance and back EMF constant
                double resistance = 0.1; // Example, get from DCMotorExt
                double backEMFConstant = 0.012; // Example, get from DCMotorExt
                double backEMF = backEMFConstant * velocityRadPerSec;
                double calculatedVoltage = backEMF + desiredCurrent.baseUnitMagnitude() * resistance;
                calculatedVoltage = Math.min(calculatedVoltage, RobotController.getBatteryVoltage()); // Clamp to supply
                talonFX.setControl(new VoltageOut(calculatedVoltage));
                double motorVoltage = talonFXSim.getMotorVoltage();
                motorSim.setInputVoltage(motorVoltage);
            }

            // Update simulated motor
            motorSim.update(dt);

            // Set new state back to CTRE SimState
            talonFXSim.setRawRotorPosition(motorSim.getAngularPosition());
            talonFXSim.setRotorVelocity(motorSim.getAngularVelocity());

            try {
                Thread.sleep((long) (dt * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}