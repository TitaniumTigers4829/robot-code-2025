package frc.robot.sim.simController;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.sim.simController.SimMotorController.ControllerOutput;
import frc.robot.sim.simController.SimMotorController.ControllerOutput.CurrentOutput;
import frc.robot.sim.simController.SimMotorController.ControllerOutput.VoltageOutput;
import frc.robot.sim.simMechanism.SimMechanism.MechanismState;

public class MotorSimRunner {
  private final UnitSafeMotorController controller;
  private final TalonFX talonFX;
  private final TalonFXSimState simState;
  private double lastVelocity = 0.0;
  private static final double STEP_TIME = 0.02; // 20ms

  public MotorSimRunner(UnitSafeMotorController controller, TalonFX fx) {
    this.controller = controller;
    this.talonFX = fx;
    this.simState = fx.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
  }

  public void simulate() {
    while (true) {
      // Get current motor state
      double position = controller.position().in(Radians); // Radians or your unit
      double velocity = controller.velocity().in(RadiansPerSecond); // Rad/s or your unit
      double acceleration = (velocity - lastVelocity) / STEP_TIME;
      lastVelocity = velocity;

      MechanismState state =
          new MechanismState(
              Angle.ofBaseUnits(position, Units.Radians),
              AngularVelocity.ofBaseUnits(velocity, Units.RadiansPerSecond),
              AngularAcceleration.ofBaseUnits(acceleration, Units.RadiansPerSecondPerSecond));

      // Run controller to get output
      ControllerOutput output =
          controller.run(
              Time.ofBaseUnits(0.02, Units.Seconds),
              Voltage.ofBaseUnits(RobotController.getBatteryVoltage(), Units.Volts),
              state);

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
      controller.controlVoltage(Volts.of(motorVoltage));
      // simState.(STEP_TIME);

      // Sync sim state back to CTRE
      simState.setRawRotorPosition(controller.position());
      simState.setRotorVelocity(controller.velocity());

      // Chill for 20ms
      try {
        Thread.sleep((long) (STEP_TIME * 1000));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
