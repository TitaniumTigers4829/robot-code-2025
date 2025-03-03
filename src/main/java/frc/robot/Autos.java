package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.elevator.IntakeCoral;
import frc.robot.commands.elevator.ScoreL4;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autos {
  private final AutoFactory autoFactory;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private SwerveDrive swerveDrive;

  public Autos(AutoFactory autoFactory) {
    this.autoFactory = autoFactory;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveDrive = swerveDrive;
  
  }

  Trigger isCoralScored = new Trigger(() -> !coralIntakeSubsystem.hasCoral());
  Trigger hasCoral = new Trigger(() -> coralIntakeSubsystem.hasCoral());
  Trigger reefInRange =  swerveDrive.isReefInRange();
  // Blue Auto Routines

  public AutoRoutine blueTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);
    
        
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
                startToETraj.done().onTrue(eToPickupTraj.cmd());
                eToPickupTraj.done().and(this.isCoralScored).onTrue(pickupToCTraj.cmd());
          
    routine
        .anyDone(startToETraj, pickupToCTraj).and(reefInRange)
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj).and(isCoralScored)
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));
    return routine;
  }

  public AutoRoutine blueThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);
    
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
                startToETraj.done().onTrue(eToPickupTraj.cmd());
                eToPickupTraj.done().and(this.isCoralScored).onTrue(pickupToCTraj.cmd());
                pickupToCTraj.done().and(hasCoral).onTrue(cToPickupTraj.cmd());
                cToPickupTraj.done().and(isCoralScored).onTrue(pickupToDTraj.cmd());
               
    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj).and(reefInRange)
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj, cToPickupTraj).and(isCoralScored)
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));
    return routine;
  }

  public AutoRoutine blueFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToBTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_B_TRAJECTORY);
    
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
    startToETraj.done().onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(this.isCoralScored).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().and(hasCoral).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().and(isCoralScored).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.done().and(hasCoral).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.done().and(isCoralScored).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj, pickupToBTraj).and(reefInRange)
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj, cToPickupTraj, dToPickupTraj).and(isCoralScored)
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  // Red Auto Routines

  public AutoRoutine redTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
                startToETraj.done().onTrue(eToPickupTraj.cmd());
                eToPickupTraj.done().and(this.isCoralScored).onTrue(pickupToCTraj.cmd());
             
    routine
        .anyDone(startToETraj, pickupToCTraj).and(reefInRange)
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj).and(isCoralScored)
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));
    return routine;
  }

  public AutoRoutine redThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.RED_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    
        
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
                startToETraj.done().onTrue(eToPickupTraj.cmd());
                eToPickupTraj.done().and(this.isCoralScored).onTrue(pickupToCTraj.cmd());
                pickupToCTraj.done().and(hasCoral).onTrue(cToPickupTraj.cmd());
                cToPickupTraj.done().and(isCoralScored).onTrue(pickupToDTraj.cmd());
             
    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj).and(reefInRange)
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj, cToPickupTraj).and(isCoralScored)
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToETraj = routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTraj =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.RED_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.RED_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToBTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_B_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                startToETraj.cmd()));
                startToETraj.done().onTrue(eToPickupTraj.cmd());
                eToPickupTraj.done().and(this.isCoralScored).onTrue(pickupToCTraj.cmd());
                pickupToCTraj.done().and(hasCoral).onTrue(cToPickupTraj.cmd());
                cToPickupTraj.done().and(isCoralScored).onTrue(pickupToDTraj.cmd());
                pickupToDTraj.done().and(hasCoral).onTrue(dToPickupTraj.cmd());
                dToPickupTraj.done().and(isCoralScored).onTrue(pickupToBTraj.cmd());
            

    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj, pickupToBTraj).and(reefInRange)
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj, cToPickupTraj, dToPickupTraj).and(isCoralScored)
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }
}
