package frc.robot.subsystems.funnelPivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HardwareConstants;

public class SimulatedFunnelPivot implements FunnelPivotInterface {
  private final double funnelGearing = FunnelConstants.FUNNEL_GEAR_RATIO;
  private final double funnelPivotMass = FunnelConstants.FUNNEL_PIVOT_MASS;
  private final double funnelPivotLength = FunnelConstants.FUNNEL_PIVOT_LENGTH;
  private SingleJointedArmSim funnelPivotSim =
      new SingleJointedArmSim(DCMotor.getKrakenX60(2), funnelGearing, 1, 1, 0, 0, true, 0);

  private final double armKS = 0.0;
  private final double armKG = FunnelConstants.PIVOT_G;
  private final double armKV = 0.0;
  private final ArmFeedforward armFeedForward = new ArmFeedforward(armKS, armKG, armKV);
  private final Constraints funnelConstraints = new Constraints(0, 0);
  private final ProfiledPIDController funnelPivotController =
      new ProfiledPIDController(0, 0, 0, funnelConstraints);

  private double appliedVolts = 0.0;
  private double position = 0.0;
  private double velocity = 0.0;
  private double supplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(FunnelPivotInputs inputs) {
    funnelPivotSim.update(HardwareConstants.LOOP_TIME_SECONDS);

    inputs.funnelVelocity = Units.radiansToRotations(funnelPivotSim.getAngleRads());
    inputs.funnelAngle = Units.radiansToRotations(funnelPivotSim.getVelocityRadPerSec());
    inputs.funnelSupplyCurrentAmps = funnelPivotSim.getCurrentDrawAmps();
    inputs.funnelVoltage = appliedVolts;
  }

  @Override
  public void setFunnelVoltage(double voltage) {
    appliedVolts = voltage;
    funnelPivotSim.setInputVoltage(voltage);
  }

  @Override
  public void setFunnelAngle(double angle) {
    double currentFunnelPivotAngleRots = Units.radiansToRotations(funnelPivotSim.getAngleRads());
    double armFF = armFeedForward.calculate(angle, funnelPivotController.getSetpoint().velocity);
    setFunnelVoltage(funnelPivotController.calculate(currentFunnelPivotAngleRots, angle) + armFF);
  }
}
