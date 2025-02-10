package frc.robot;

import java.util.List;
import java.util.Optional;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;

public class FlexiAutos {
  private final AutoFactory autoFactory;
  private final List nodes;
  public FlexiAutos(AutoFactory autoFactory, List nodes) {
    this.autoFactory = autoFactory;
    this.nodes = nodes;

  }

  public AutoRoutine flexiRoutine() {

    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.FLEXIBLE_AUTO_ROUTINE);

    AutoTrajectory leftStartToJ = routine.trajectory(AutoConstants.LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory rightStartToE = routine.trajectory(AutoConstants.RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory middleStartToG = routine.trajectory(AutoConstants.MID_START_TO_G_TRAJECTORY);
    AutoTrajectory middleStartToH = routine.trajectory(AutoConstants.MID_START_TO_H_TRAJECTORY);
    
    AutoTrajectory aToLeftPickup = routine.trajectory(AutoConstants.A_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory bToLeftPickup = routine.trajectory(AutoConstants.B_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory jToLeftPickup = routine.trajectory(AutoConstants.J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory lToLeftPickup = routine.trajectory(AutoConstants.L_TO_LEFT_PICKUP_TRAJECTORY);
   
    AutoTrajectory aToRightPickup = routine.trajectory(AutoConstants.A_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory bToRightPickup = routine.trajectory(AutoConstants.B_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory cToRightPickup = routine.trajectory(AutoConstants.C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory dToRightPickup = routine.trajectory(AutoConstants.D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory eToRightPickup = routine.trajectory(AutoConstants.E_TO_RIGHT_PICKUP_TRAJECTORY);

    AutoTrajectory leftPickupToA = routine.trajectory(AutoConstants.LEFT_PICKUP_TO_A_TRAJECTORY);
    AutoTrajectory leftPickupToB = routine.trajectory(AutoConstants.LEFT_PICKUP_TO_B_TRAJECTORY);
    AutoTrajectory leftPickupToJ = routine.trajectory(AutoConstants.LEFT_PICKUP_TO_J_TRAJECTORY);
    AutoTrajectory leftPickupToL = routine.trajectory(AutoConstants.LEFT_PICKUP_TO_L_TRAJECTORY);
    
    AutoTrajectory rightPickupToA = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_A_TRAJECTORY);
    AutoTrajectory rightPickupToB = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_B_TRAJECTORY);
    AutoTrajectory rightPickupToC = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory rightPickupToD = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory rightPickupToE = routine.trajectory(AutoConstants.RIGHT_PICKUP_TO_E_TRAJECTORY);

    //TODO: logic

    AutoTrajectory firstPath = rightPickupToA;
    Optional<Pose2d> startingPose = firstPath.getInitialPose();
    routine.active(

    ).onTrue(
      autoFactory.resetOdometry("startingPose")
      
    );

    return routine;

  }
}
