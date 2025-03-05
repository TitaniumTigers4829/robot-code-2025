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
  private final double algaeGearing = AlgaePivotConstants.ALGAE_GEAR_RATIO;
  private final double algaePivotMass = AlgaePivotConstants.ALGAE_MOMENT_INERTIA;
  private final double algaePivotLength = AlgaePivotConstants.ALGAE_PIVOT_LENGTH;
  private SingleJointedArmSim algaePivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          algaeGearing,
          algaePivotMass,
          algaePivotLength,
          Radians.of(AlgaePivotConstants.MIN_ANGLE).in(Rotations),
          Radians.of(AlgaePivotConstants.MAX_ANGLE).in(Rotations),
          true,
          Radians.of(AlgaePivotConstants.ANGLE_ZERO).in(Rotations));

  private final double armKS = 0.0;
  private final double armKG = AlgaePivotConstants.PIVOT_G;
  private final double armKV = 0.0;
  private final ArmFeedforward armFeedForward = new ArmFeedforward(armKS, armKG, armKV);
  private final Constraints algaeConstraints =
      new Constraints(
          AlgaePivotConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND,
          AlgaePivotConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND);
  private final ProfiledPIDController algaePivotController =
      new ProfiledPIDController(
          Radians.of(AlgaePivotConstants.PIVOT_P).in(Rotations),
          Radians.of(AlgaePivotConstants.PIVOT_I).in(Rotations),
          Radians.of(AlgaePivotConstants.PIVOT_D).in(Rotations),
          algaeConstraints);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(AlgaePivotInputs inputs) {
    algaePivotSim.update(HardwareConstants.LOOP_TIME_SECONDS);

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
