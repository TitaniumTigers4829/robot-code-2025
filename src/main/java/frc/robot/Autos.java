package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autodrive.RepulsorReef;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.IntakeCoral;
import frc.robot.commands.elevator.ScoreL4;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
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

  Trigger elevatorUpZone = new Trigger(() -> swerveDrive.isReefInRange());
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

  public AutoRoutine blueLeftTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_LEFT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory = routine.trajectory("RedTrajectories/new-path");
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_A_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_LEFT_START_TO_J_TRAJECTORY),
                // funnelSubsystem.dropFunnel(),
                // elevatorSubsystem.manualElevator(-0.2).withTimeout(0.2),
                // Commands.runOnce(() -> elevatorSubsystem.resetPosition(0.0), elevatorSubsystem),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(2.0)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, true)
                        .alongWith(
                            new SetElevatorPosition(
                                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(4.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0))));

    Logger.recordOutput("Trajectories/starttoj", startToJTrajectory.getRawTrajectory().getPoses());
    Logger.recordOutput(
        "Trajectories/jtopickup", jToPickupTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    // "Trajectories/pickuptoL", pickupToLTrajectory.getRawTrajectory().getPoses());
    // startToJTrajectory.done().onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    // startToJTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToLTrajectory.cmd());

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(leftReefInRange).negate())
    //     .onTrue(
    //         Commands.sequence(
    //             new RepulsorReef(swerveDrive, visionSubsystem, true),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Repulsor Auto Trigger", true))));

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(hasCoral))
    //     .and(routine.observe(rightReefInRange))
    //     .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .anyActive(jToPickupTrajectory)
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .observe(elevatorUpZone)
    //     .and(routine.observe(hasCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Elevator Up Auto Trigger",
    // true))));
    // routine
    //     .observe(elevatorUpZone.negate())
    //     .or(routine.observe(hasNoCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem,
    // ElevatorSetpoints.FEEDER.getPosition()),
    //             new RunCommand(
    //                 () -> SmartDashboard.putBoolean("Elevator Up Auto Trigger", false))));
    return routine;
  }

  public AutoRoutine blueRightTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_RIGHT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                // funnelSubsystem.dropFunnel(),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(2.0)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, false)
                        .alongWith(
                            new SetElevatorPosition(
                                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(4.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0))));

    Logger.recordOutput("Trajectories/starttoj", startToJTrajectory.getRawTrajectory().getPoses());
    Logger.recordOutput(
        "Trajectories/jtopickup", jToPickupTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    // "Trajectories/pickuptoL", pickupToLTrajectory.getRawTrajectory().getPoses());
    // startToJTrajectory.done().onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    // startToJTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToLTrajectory.cmd());

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(leftReefInRange).negate())
    //     .onTrue(
    //         Commands.sequence(
    //             new RepulsorReef(swerveDrive, visionSubsystem, true),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Repulsor Auto Trigger", true))));

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(hasCoral))
    //     .and(routine.observe(rightReefInRange))
    //     .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .anyActive(jToPickupTrajectory)
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .observe(elevatorUpZone)
    //     .and(routine.observe(hasCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Elevator Up Auto Trigger",
    // true))));
    // routine
    //     .observe(elevatorUpZone.negate())
    //     .or(routine.observe(hasNoCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem,
    // ElevatorSetpoints.FEEDER.getPosition()),
    //             new RunCommand(
    //                 () -> SmartDashboard.putBoolean("Elevator Up Auto Trigger", false))));
    return routine;
  }

  public AutoRoutine blueThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
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
                startToJTrajectory.cmd()));
    startToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToLTrajectory.cmd());
    pickupToLTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToDTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory, pickupToDTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(jToPickupTrajectory, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .observe(elevatorUpZone)
        .and(routine.observe(hasCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()));
    routine
        .observe(elevatorUpZone.negate())
        .or(routine.observe(hasNoCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.FEEDER.getPosition()));
    return routine;
  }

  public AutoRoutine blueFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
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
                startToJTrajectory.cmd()));
    startToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToLTrajectory.cmd());
    pickupToLTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTraj, pickupToBTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory, pickupToDTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(jToPickupTrajectory, cToPickupTraj, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .observe(elevatorUpZone)
        .and(routine.observe(hasCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()));
    routine
        .observe(elevatorUpZone.negate())
        .or(routine.observe(hasNoCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.FEEDER.getPosition()));
    return routine;
  }

  // Red Auto Routines

  //   public AutoRoutine redTwoCoralAuto() {
  //     AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_TWO_CORAL_AUTO_ROUTINE);

  //     AutoTrajectory startToJTrajectory =
  //         routine.trajectory(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY);
  //     AutoTrajectory jToPickupTrajectory =
  //         routine.trajectory(AutoConstants.RED_J_TO_LEFT_PICKUP_TRAJECTORY);
  //     AutoTrajectory pickupToLTrajectory =
  //         routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_L_TRAJECTORY);

  //     routine
  //         .active()
  //         .onTrue(
  //             Commands.sequence(
  //                 autoFactory.resetOdometry(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY),
  //                 startToJTrajectory.cmd()));
  //     startToJTrajectory
  //         .recentlyDone()
  //         .and(routine.observe(hasNoCoral))
  //         .onTrue(jToPickupTrajectory.cmd());
  //     jToPickupTrajectory
  //         .recentlyDone()
  //         .and(routine.observe(hasCoral))
  //         .onTrue(pickupToLTrajectory.cmd());

  //     routine
  //         .anyDone(startToJTrajectory, pickupToLTrajectory)
  //         .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));

  //     routine
  //         .anyDone(startToJTrajectory, pickupToLTrajectory)
  //         .and(routine.observe(hasCoral))
  //         .and(routine.observe(leftReefInRange))
  //         .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
  //     routine
  //         .anyActive(jToPickupTrajectory)
  //         .and(routine.observe(hasNoCoral))
  //         .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

  //     routine
  //         .observe(elevatorUpZone)
  //         .and(routine.observe(hasCoral))
  //         .onTrue(
  //             new SetElevatorPosition(
  //                 swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()));
  //     routine
  //         .observe(elevatorUpZone.negate())
  //         .or(routine.observe(hasNoCoral))
  //         .onTrue(
  //             new SetElevatorPosition(
  //                 swerveDrive, elevatorSubsystem, ElevatorSetpoints.FEEDER.getPosition()));
  //     return routine;
  //   }
  public AutoRoutine redLeftTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_LEFT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
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
                // funnelSubsystem.dropFunnel(),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(2.0)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, false)
                        .alongWith(
                            new SetElevatorPosition(
                                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(4.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0))));

    Logger.recordOutput("Trajectories/starttoj", startToJTrajectory.getRawTrajectory().getPoses());
    Logger.recordOutput(
        "Trajectories/jtopickup", jToPickupTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    // "Trajectories/pickuptoL", pickupToLTrajectory.getRawTrajectory().getPoses());
    // startToJTrajectory.done().onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    // startToJTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToLTrajectory.cmd());

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(leftReefInRange).negate())
    //     .onTrue(
    //         Commands.sequence(
    //             new RepulsorReef(swerveDrive, visionSubsystem, true),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Repulsor Auto Trigger", true))));

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(hasCoral))
    //     .and(routine.observe(rightReefInRange))
    //     .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .anyActive(jToPickupTrajectory)
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .observe(elevatorUpZone)
    //     .and(routine.observe(hasCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Elevator Up Auto Trigger",
    // true))));
    // routine
    //     .observe(elevatorUpZone.negate())
    //     .or(routine.observe(hasNoCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem,
    // ElevatorSetpoints.FEEDER.getPosition()),
    //             new RunCommand(
    //                 () -> SmartDashboard.putBoolean("Elevator Up Auto Trigger", false))));
    return routine;
  }

  public AutoRoutine redRightTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_RIGHT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                // funnelSubsystem.dropFunnel(),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(2.0)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, true)
                        .alongWith(
                            new SetElevatorPosition(
                                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(4.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0))));

    Logger.recordOutput("Trajectories/starttoj", startToJTrajectory.getRawTrajectory().getPoses());
    Logger.recordOutput(
        "Trajectories/jtopickup", jToPickupTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    // "Trajectories/pickuptoL", pickupToLTrajectory.getRawTrajectory().getPoses());
    // startToJTrajectory.done().onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    // startToJTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToLTrajectory.cmd());

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(leftReefInRange).negate())
    //     .onTrue(
    //         Commands.sequence(
    //             new RepulsorReef(swerveDrive, visionSubsystem, true),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Repulsor Auto Trigger", true))));

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(hasCoral))
    //     .and(routine.observe(rightReefInRange))
    //     .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .anyActive(jToPickupTrajectory)
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .observe(elevatorUpZone)
    //     .and(routine.observe(hasCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Elevator Up Auto Trigger",
    // true))));
    // routine
    //     .observe(elevatorUpZone.negate())
    //     .or(routine.observe(hasNoCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem,
    // ElevatorSetpoints.FEEDER.getPosition()),
    //             new RunCommand(
    //                 () -> SmartDashboard.putBoolean("Elevator Up Auto Trigger", false))));
    return routine;
  }

  public AutoRoutine redThreeCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_THREE_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
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
                startToJTrajectory.cmd()));
    startToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToLTrajectory.cmd());
    pickupToLTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine.anyDone(pickupToDTraj).onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory, pickupToDTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
    routine
        .anyActive(jToPickupTrajectory, cToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .observe(elevatorUpZone)
        .and(routine.observe(hasCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()));
    routine
        .observe(elevatorUpZone.negate())
        .or(routine.observe(hasNoCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.FEEDER.getPosition()));
    return routine;
  }

  public AutoRoutine redFourCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_FOUR_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
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
                startToJTrajectory.cmd()));
    startToJTrajectory
        .recentlyDone()
        .and(routine.observe(hasNoCoral))
        .onTrue(jToPickupTrajectory.cmd());
    jToPickupTrajectory
        .recentlyDone()
        .and(routine.observe(hasCoral))
        .onTrue(pickupToLTrajectory.cmd());
    pickupToLTrajectory.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(cToPickupTraj.cmd());
    cToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToDTraj.cmd());
    pickupToDTraj.recentlyDone().and(routine.observe(hasNoCoral)).onTrue(dToPickupTraj.cmd());
    dToPickupTraj.recentlyDone().and(routine.observe(hasCoral)).onTrue(pickupToBTraj.cmd());

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
    routine
        .anyDone(pickupToDTraj, pickupToBTraj)
        .onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));

    routine
        .anyDone(startToJTrajectory, pickupToLTrajectory, pickupToDTraj, pickupToBTraj)
        .and(routine.observe(hasCoral))
        .and(routine.observe(leftReefInRange))
        .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .anyActive(jToPickupTrajectory, cToPickupTraj, dToPickupTraj)
        .and(routine.observe(hasNoCoral))
        .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    routine
        .observe(elevatorUpZone)
        .and(routine.observe(hasCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.L4.getPosition()));
    routine
        .observe(elevatorUpZone.negate())
        .or(routine.observe(hasNoCoral))
        .onTrue(
            new SetElevatorPosition(
                swerveDrive, elevatorSubsystem, ElevatorSetpoints.FEEDER.getPosition()));
    return routine;
  }
}
