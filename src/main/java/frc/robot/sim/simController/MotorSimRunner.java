package frc.robot.sim.simController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorSimRunner {
    private final UnitSafeMotorController controller;
    private final TalonFX talonFX;
    private final TalonFXSimState simState;
    private final DCMotorSim motorSim;
    private double lastVelocity = 0.0;
    private static final double STEP_TIME = 0.02; // 20ms

    public MotorSimRunner(UnitSafeMotorController controller, TalonFX fx, DCMotorSim sim) {
        this.controller = controller;
        this.talonFX = fx;
        this.simState = fx.getSimState();
        this.motorSim = sim;
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public void simulate() {
        while (true) {
            // Get current motor state
            double position = motorSim.getAngularPositionRad(); // Radians or your unit
            double velocity = motorSim.getAngularVelocityRadPerSec(); // Rad/s or your unit
            double acceleration = (velocity - lastVelocity) / STEP_TIME;
            lastVelocity = velocity;

            MechanismState state = new MechanismState(
                Angle.of(position),
                AngularVelocity.of(velocity),
                AngularAcceleration.of(acceleration)
            );

            // Run controller to get output
            ControllerOutput output = controller.run(
                Time.of(STEP_TIME, Units.Seconds),
                Voltage.of(RobotController.getBatteryVoltage()),
                state
            );

            // Apply output to TalonFX
            if (output instanceof VoltageOutput vo) {
                talonFX.setControl(new VoltageOut(vo.voltage().baseUnitMagnitude()));
            } else if (output instanceof CurrentOutput co) {
                // Approximate voltage for current (no Pro license workaround)
                double resistance = 0.1; // Tune this to your motor
                double backEMFConst = 0.012; // Tune this too
                double backEMF = velocity * backEMFConst;
                double voltage = backEMF + co.current().baseUnitMagnitude() * resistance;
                voltage = Math.min(voltage, RobotController.getBatteryVoltage());
                talonFX.setControl(new VoltageOut(voltage));
            }

            // Update motor sim with TalonFX voltage
            double motorVoltage = simState.getMotorVoltage();
            motorSim.setInputVoltage(motorVoltage);
            motorSim.update(STEP_TIME);

            // Sync sim state back to CTRE
            simState.setRawRotorPosition(motorSim.getAngle());
            simState.setRotorVelocity(motorSim.getAngularVelocity());

            // Chill for 20ms
            try {
                Thread.sleep((long) (STEP_TIME * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}