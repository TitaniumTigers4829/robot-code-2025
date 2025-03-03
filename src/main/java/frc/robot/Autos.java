package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autodrive.RepulsorReef;
import frc.robot.commands.elevator.IntakeCoral;
import frc.robot.commands.elevator.ScoreL4;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Autos {
  private final AutoFactory autoFactory;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private SwerveDrive swerveDrive;
  private VisionSubsystem visionSubsystem;

  public Autos(
      AutoFactory autoFactory,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem) {
    this.autoFactory = autoFactory;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
  }

  Trigger hasCoral = new Trigger(() -> coralIntakeSubsystem.hasCoral());
  Trigger hasNoCoral = hasCoral.negate();

  Trigger leftReefInRange = new Trigger(() -> swerveDrive.isRobotAlignedToLeftReef());
  Trigger rightReefInRange = new Trigger(() -> swerveDrive.isRobotAlignedToRightReef());

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
    startToETraj.done().and(routine.observe(hasNoCoral)).onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .and(routine.observe(leftReefInRange).negate())
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj)
        .and(routine.observe(hasCoral))
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
    startToETraj.done().and(routine.observe(hasNoCoral)).onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToDTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTraj, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
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
    startToETraj.done().and(routine.observe(hasNoCoral)).onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.done().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTraj, pickupToBTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTraj, cToPickupTraj, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
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
    startToETraj.done().and(routine.observe(hasNoCoral)).onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj)
        .and(routine.observe(hasNoCoral))
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
    startToETraj.done().and(routine.observe(hasNoCoral)).onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToDTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTraj, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
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
    startToETraj.done().and(routine.observe(hasNoCoral)).onTrue(eToPickupTraj.cmd());
    eToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.done().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.done().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.done().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(startToETraj, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTraj, pickupToBTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToETraj, pickupToCTraj, pickupToDTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTraj, cToPickupTraj, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }
}
