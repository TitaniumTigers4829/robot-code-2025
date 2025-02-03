package frc.robot.subsystems.AlgaePivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulatedAlgaePivot implements AlgaePivotInterface{
  private final double algaeGearing = AlgaeConstants.ALGAE_GEAR_RATIO;
  private final double algaePivotMass = AlgaeConstants.ALGAE_PIVOT_MASS;
  private final double algaePivotLength = AlgaeConstants.ALGAE_PIVOT_LENGTH;
}
