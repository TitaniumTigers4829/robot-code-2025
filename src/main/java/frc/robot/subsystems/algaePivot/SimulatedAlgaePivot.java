package frc.robot.subsystems.algaePivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HardwareConstants;

public class SimulatedAlgaePivot implements AlgaePivotInterface {
  private final double algaeGearing = AlgaeConstants.ALGAE_GEAR_RATIO;
  private final double algaePivotMass = AlgaeConstants.ALGAE_MOMENT_INERTIA;
  private final double algaePivotLength = AlgaeConstants.ALGAE_PIVOT_LENGTH;
  private SingleJointedArmSim algaePivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          algaeGearing,
          algaePivotMass,
          algaePivotLength,
          Radians.of(AlgaeConstants.MIN_ANGLE).in(Rotations),
          Radians.of(AlgaeConstants.MAX_ANGLE).in(Rotations),
          true,
          Radians.of(AlgaeConstants.ANGLE_ZERO).in(Rotations));

  private final double armKS = 0.0;
  private final double armKG = AlgaeConstants.PIVOT_G;
  private final double armKV = 0.0;
  private final ArmFeedforward armFeedForward = new ArmFeedforward(armKS, armKG, armKV);
  private final Constraints algaeConstraints =
      new Constraints(
          AlgaeConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND,
          AlgaeConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND);
  private final ProfiledPIDController algaePivotController =
      new ProfiledPIDController(
          AlgaeConstants.PIVOT_P, AlgaeConstants.PIVOT_I, AlgaeConstants.PIVOT_D, algaeConstraints);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(AlgaePivotInputs inputs) {
    algaePivotSim.update(HardwareConstants.TIMEOUT_S);

    inputs.algaeVelocity = Units.radiansToRotations(algaePivotSim.getVelocityRadPerSec());
    inputs.algaeAngle = Units.radiansToRotations(algaePivotSim.getAngleRads());
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
    setAlgaeVoltage(algaePivotController.calculate(currentAlgaePivotAngleRots, angle) + armFF);
  }
}
