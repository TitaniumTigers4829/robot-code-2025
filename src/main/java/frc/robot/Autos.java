package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autodrive.RepulsorReef;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.IntakeCoral;
import frc.robot.commands.elevator.ScoreL4;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final AutoFactory autoFactory;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private SwerveDrive swerveDrive;
  private VisionSubsystem visionSubsystem;
  private FunnelSubsystem funnelSubsystem;

  public Autos(
      AutoFactory autoFactory,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      FunnelSubsystem funnelSubsystem) {
    this.autoFactory = autoFactory;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    this.funnelSubsystem = funnelSubsystem;
  }

  Trigger hasCoral = new Trigger(() -> coralIntakeSubsystem.hasCoral());
  Trigger hasNoCoral = hasCoral.negate();

  Trigger leftReefInRange = new Trigger(() -> swerveDrive.isRobotAlignedToLeftReef());
  Trigger rightReefInRange = new Trigger(() -> swerveDrive.isRobotAlignedToRightReef());

  public AutoRoutine xOneMeterAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.X_ONE_METER_AUTO);
    AutoTrajectory xOneMeterTrajectory = routine.trajectory(AutoConstants.X_ONE_METER_TRAJECTORY);
    Logger.recordOutput("two meter auto", xOneMeterTrajectory.getRawTrajectory().getPoses());
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.X_ONE_METER_TRAJECTORY),
                xOneMeterTrajectory.cmd()));

    return routine;
  }

  public AutoRoutine yOneMeterAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.Y_ONE_METER_AUTO);
    AutoTrajectory yOneMeterTrajectory = routine.trajectory(AutoConstants.Y_ONE_METER_TRAJECTORY);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.Y_ONE_METER_TRAJECTORY),
                yOneMeterTrajectory.cmd()));

    return routine;
  }

  // Blue Auto Routines
  public AutoRoutine simpleRepulsorAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.SIMPLE_REPULSOR_AUTO);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())),
                new DriveCommand(
                        swerveDrive,
                        visionSubsystem,
                        () -> -0.15,
                        () -> 0.0,
                        () -> 0,
                        () -> false,
                        () -> false)
                    .withTimeout(3.5),
                new RepulsorReef(swerveDrive, visionSubsystem, true).withTimeout(4),
                new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)));
    return routine;
  }

  public AutoRoutine blueRightTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_RIGHT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory rStartToETrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                rStartToETrajectory.cmd()));

    rStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());

    routine
        .anyDone(rStartToETrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(rStartToETrajectory, pickupToDTrajectory)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueMidTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_MID_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory mStartToETrajectory =
        routine.trajectory(AutoConstants.BLUE_MID_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_MID_START_TO_E_TRAJECTORY),
                mStartToETrajectory.cmd()));

    mStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToCTrajectory.cmd());

    routine
        .anyDone(mStartToETrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToCTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(mStartToETrajectory, pickupToCTrajectory)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueLeftTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_LEFT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToKTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_K_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_LEFT_START_TO_J_TRAJECTORY),
                lStartToJTrajectory.cmd()));

    lStartToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToKTrajectory.cmd());

    routine
        .anyDone(lStartToJTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToKTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(lStartToJTrajectory, pickupToKTrajectory)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(jToPickupTrajectory)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueRightThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_RIGHT_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory rStartToETrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTrajectory =
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
                rStartToETrajectory.cmd()));
    rStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToCTrajectory.cmd());
    pickupToCTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());

    routine
        .anyDone(rStartToETrajectory, pickupToCTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToDTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(rStartToETrajectory, pickupToCTrajectory, pickupToDTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueMidThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_MID_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory mStartToETrajectory =
        routine.trajectory(AutoConstants.BLUE_MID_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTraj =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_MID_START_TO_E_TRAJECTORY),
                mStartToETrajectory.cmd()));
    mStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToCTrajectory.cmd());
    pickupToCTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());

    routine
        .anyDone(mStartToETrajectory, pickupToCTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToDTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(mStartToETrajectory, pickupToCTrajectory, pickupToDTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueLeftThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_LEFT_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToITrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_START_TO_I_TRAJECTORY);
    AutoTrajectory iToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_I_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToKTraj =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_K_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_LEFT_START_TO_I_TRAJECTORY),
                lStartToITrajectory.cmd()));
    lStartToITrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(iToPickupTrajectory.cmd());
    iToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToJTrajectory.cmd());
    pickupToJTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(jToPickupTraj.cmd());
    jToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToKTraj.cmd());

    routine
        .anyDone(lStartToITrajectory, pickupToKTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToKTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(lStartToITrajectory, pickupToJTrajectory, pickupToKTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(iToPickupTrajectory, jToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueRightFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_RIGHT_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory rStartToETrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTrajectory =
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
                rStartToETrajectory.cmd()));
    rStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToCTrajectory.cmd());
    pickupToCTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(rStartToETrajectory, pickupToCTrajectory, pickupToDTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToBTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(rStartToETrajectory, pickupToCTrajectory, pickupToDTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory, cToPickupTraj, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueMidFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_MID_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory mStartToETrajectory =
        routine.trajectory(AutoConstants.BLUE_MID_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTrajectory =
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
                autoFactory.resetOdometry(AutoConstants.BLUE_MID_START_TO_E_TRAJECTORY),
                mStartToETrajectory.cmd()));
    mStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToCTrajectory.cmd());
    pickupToCTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(mStartToETrajectory, pickupToCTrajectory, pickupToDTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToBTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(mStartToETrajectory, pickupToCTrajectory, pickupToDTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory, cToPickupTraj, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueLeftFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_LEFT_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToITrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_START_TO_I_TRAJECTORY);
    AutoTrajectory iToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_I_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToKTraj =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_K_TRAJECTORY);
    AutoTrajectory kToPickupTraj =
        routine.trajectory(AutoConstants.BLUE_K_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTraj =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_L_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_LEFT_START_TO_I_TRAJECTORY),
                lStartToITrajectory.cmd()));
    lStartToITrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(iToPickupTrajectory.cmd());
    iToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToJTrajectory.cmd());
    pickupToJTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(jToPickupTraj.cmd());
    jToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToKTraj.cmd());
    pickupToKTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(kToPickupTraj.cmd());
    kToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToLTraj.cmd());

    routine
        .anyDone(lStartToITrajectory, pickupToJTrajectory, pickupToKTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToLTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(lStartToITrajectory, pickupToJTrajectory, pickupToKTraj, pickupToLTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(iToPickupTrajectory, jToPickupTraj, kToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  // Red Auto Routines

  public AutoRoutine redLeftTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_LEFT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToJTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_H_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY),
                lStartToJTrajectory.cmd()));
    lStartToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToLTrajectory.cmd());

    routine
        .anyDone(lStartToJTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToLTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(lStartToJTrajectory, pickupToLTrajectory)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(jToPickupTrajectory)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redMidTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_MID_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory mStartToETrajectory =
        routine.trajectory(AutoConstants.RED_MID_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_MID_START_TO_E_TRAJECTORY),
                mStartToETrajectory.cmd()));
    mStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());

    routine
        .anyDone(mStartToETrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(mStartToETrajectory, pickupToDTrajectory)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTrajectory)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redRightTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_RIGHT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory rStartToETrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                rStartToETrajectory.cmd()));
    rStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());

    routine
        .anyDone(rStartToETrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(rStartToETrajectory, pickupToDTrajectory)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTrajectory)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redLeftThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_LEFT_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToJTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToKTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_K_TRAJECTORY);
    AutoTrajectory kToPickupTraj =
        routine.trajectory(AutoConstants.RED_K_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTraj =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_L_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY),
                lStartToJTrajectory.cmd()));
    lStartToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToKTrajectory.cmd());
    pickupToKTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(kToPickupTraj.cmd());
    kToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToLTraj.cmd());

    routine
        .anyDone(lStartToJTrajectory, pickupToKTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToLTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(lStartToJTrajectory, pickupToKTrajectory, pickupToLTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(jToPickupTrajectory, kToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redMidThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_MID_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory mStartToETrajectory =
        routine.trajectory(AutoConstants.RED_MID_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.RED_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_MID_START_TO_E_TRAJECTORY),
                mStartToETrajectory.cmd()));
    mStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());
    pickupToDTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());

    routine
        .anyDone(mStartToETrajectory, pickupToDTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToCTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(mStartToETrajectory, pickupToDTrajectory, pickupToCTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTrajectory, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redRightThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_RIGHT_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory rStartToETrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.RED_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                rStartToETrajectory.cmd()));
    rStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());
    pickupToDTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());

    routine
        .anyDone(rStartToETrajectory, pickupToDTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToCTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(rStartToETrajectory, pickupToDTrajectory, pickupToCTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(eToPickupTrajectory, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redLeftFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_LEFT_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToJTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToKTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_K_TRAJECTORY);
    AutoTrajectory kToPickupTraj =
        routine.trajectory(AutoConstants.RED_K_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTraj =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_L_TRAJECTORY);
    AutoTrajectory lToPickupTraj =
        routine.trajectory(AutoConstants.RED_L_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToATraj =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_A_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY),
                lStartToJTrajectory.cmd()));
    lStartToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToKTrajectory.cmd());
    pickupToKTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(kToPickupTraj.cmd());
    kToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToLTraj.cmd());
    pickupToLTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(lToPickupTraj.cmd());
    lToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToATraj.cmd());

    routine
        .anyDone(lStartToJTrajectory, pickupToKTrajectory, pickupToLTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToATraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(lStartToJTrajectory, pickupToKTrajectory, pickupToLTraj, pickupToATraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(jToPickupTrajectory, kToPickupTraj, lToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redMidFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_MID_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory mStartToETrajectory =
        routine.trajectory(AutoConstants.RED_MID_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.RED_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.RED_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToBTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_B_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_MID_START_TO_E_TRAJECTORY),
                mStartToETrajectory.cmd()));
    mStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());
    pickupToDTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(mStartToETrajectory, pickupToDTrajectory, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToBTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(mStartToETrajectory, pickupToDTrajectory, pickupToCTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory, dToPickupTraj, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine redRightFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_RIGHT_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory rStartToETrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToDTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);
    AutoTrajectory dToPickupTraj =
        routine.trajectory(AutoConstants.RED_D_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToCTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_C_TRAJECTORY);
    AutoTrajectory cToPickupTraj =
        routine.trajectory(AutoConstants.RED_C_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToBTraj =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_B_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                rStartToETrajectory.cmd()));
    rStartToETrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(eToPickupTrajectory.cmd());
    eToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToDTrajectory.cmd());
    pickupToDTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToCTraj.cmd());
    pickupToCTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(rStartToETrajectory, pickupToDTrajectory, pickupToCTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToBTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(rStartToETrajectory, pickupToDTrajectory, pickupToCTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(eToPickupTrajectory, dToPickupTraj, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }
}
