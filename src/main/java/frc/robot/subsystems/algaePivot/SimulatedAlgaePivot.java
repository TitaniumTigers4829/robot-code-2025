package frc.robot.subsystems.algaePivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HardwareConstants;

public class SimulatedAlgaePivot implements AlgaePivotInterface {
  private final double algaeGearing = AlgaeConstants.ALGAE_GEAR_RATIO;
  private final double algaePivotMass = AlgaeConstants.ALGAE_PIVOT_MASS;
  private final double algaePivotLength = AlgaeConstants.ALGAE_PIVOT_LENGTH;
  private SingleJointedArmSim algaePivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2), algaeGearing, algaePivotMass, algaePivotLength, 0, 0, true, 0);

  private final double armKS = 0.0;
  private final double armKG = AlgaeConstants.PIVOT_G;
  private final double armKV = 0.0;
  private final ArmFeedforward armFeedForward = new ArmFeedforward(armKS, armKG, armKV);
  private final Constraints algaeConstraints = new Constraints(0, 0);
  private final ProfiledPIDController algaePivotController =
      new ProfiledPIDController(0, 0, 0, algaeConstraints);

  private double appliedVolts = 0.0;
  private double position = 0.0;
  private double velocity = 0.0;
  private double supplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(AlgaePivotInputs inputs) {
    algaePivotSim.update(HardwareConstants.TIMEOUT_S);

    inputs.algaeVelocity = Units.radiansToRotations(algaePivotSim.getAngleRads());
    inputs.algaeAngle = Units.radiansToRotations(algaePivotSim.getVelocityRadPerSec());
    inputs.algaeSupplyCurrentAmps = algaePivotSim.getCurrentDrawAmps();
    inputs.algaeVoltage = appliedVolts;
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    appliedVolts = voltage;
    algaePivotSim.setInputVoltage(voltage);
  }

  @Override
  public void setAlgaeAngle(double angle) {
    double currentAlgaePivotAngleRots = Units.radiansToRotations(algaePivotSim.getAngleRads());
    double armFF = armFeedForward.calculate(angle, algaePivotController.getSetpoint().velocity);
    setAlgaeVoltage(algaePivotController.calculate(angle, currentAlgaePivotAngleRots) + armFF);
  }
}
