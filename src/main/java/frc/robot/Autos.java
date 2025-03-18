package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.autodrive.RepulsorCommand;
import frc.robot.commands.autodrive.RepulsorReef;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.FollowSwerveSampleCommand;
import frc.robot.commands.elevator.IntakeCoral;
import frc.robot.commands.elevator.ScoreL4;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.ReefLocations;
import frc.robot.extras.util.ReefLocations.ReefBranch;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final LoggedDashboardChooser<String> chooser;
  private final AutoFactory autoFactory;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private SwerveDrive swerveDrive;
  private VisionSubsystem visionSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private final String NONE_NAME = "Do Nothing";

  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();

  private final Pose2d sourceRight = new Pose2d(1.61, .67, Rotation2d.fromDegrees(54));
  private final Pose2d sourceLeft =
      new Pose2d(
          sourceRight.getX(),
          FieldConstants.FIELD_WIDTH_METERS - sourceRight.getY(),
          sourceRight.getRotation().unaryMinus());

  @FunctionalInterface
  private interface ReefRepulsorCommand {
    Command goTo(ReefBranch branch);
  }

  @FunctionalInterface
  private interface SourceRepulsorCommand {
    Command goTo(Source source);
  }

  private final ReefRepulsorCommand reefPathfinding;
  private final SourceRepulsorCommand sourcePathfinding;

  private String selectedCommandName = NONE_NAME;
  private Command selectedCommand = Commands.none();
  private boolean selectedOnRed = false;

  public Autos(
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      FunnelSubsystem funnelSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    // this sets up the auto factory
    this.autoFactory =
        new AutoFactory(
            swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to the
            (SwerveSample sample) -> {
              FollowSwerveSampleCommand followSwerveSampleCommand =
                  new FollowSwerveSampleCommand(this.swerveDrive, this.visionSubsystem, sample);
              followSwerveSampleCommand.execute();
              Logger.recordOutput("Trajectory/sample", sample.getPose());
            }, // A function that follows a choreo trajectory
            false, // If alliance flipping should be enabled
            this.swerveDrive); // The drive subsystem

    reefPathfinding =
        branch ->
            new RepulsorCommand(
                    swerveDrive,
                    visionSubsystem,
                    ReefLocations.getScoringLocation(branch),
                    swerveDrive.getEstimatedPose())
                .withName("Reef Align " + branch.name());
    sourcePathfinding =
        source -> {
          Pose2d pose = source == Source.L ? sourceLeft : sourceRight;
          if (AllianceFlipper.isRed()) {
            pose = pose.rotateAround(new Pose2d(FieldConstants.FIELD_CENTER, Rotation2d.kPi));
          }
          return new RepulsorCommand(
                  swerveDrive, visionSubsystem, pose, swerveDrive.getEstimatedPose())
              .withName("Source Align " + source.name());
        };
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

  public AutoRoutine yOneMeterAndRotationAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Rotation auto");
    AutoTrajectory yOneMeterAndRotationAuto = routine.trajectory("RedTrajectories/rotatepath");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry("RedTrajectories/rotatepath"),
                yOneMeterAndRotationAuto.cmd()));

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

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_L_TRAJECTORY);

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
            //         new WaitCommand(2.0)
            //             .andThen(
            new RepulsorReef(swerveDrive, visionSubsystem, false)
                .withTimeout(2.0)
                .andThen(jToPickupTrajectory.cmd()));
    jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    pickupToLTrajectory.done().onTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    //                     .alongWith(
    //                         new SetElevatorPosition(
    //                             swerveDrive, elevatorSubsystem,
    // ElevatorSetpoints.L4.getPosition()))
    //                     .withTimeout(4.0)
    //                     .andThen(
    //                         Commands.runEnd(
    //                                 () ->
    //                                     coralIntakeSubsystem.setIntakeVelocity(
    //                                         CoralIntakeConstants.EJECT_SPEED),
    //                                 () ->
    //                                     coralIntakeSubsystem.setIntakeVelocity(
    //                                         CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
    //                                 coralIntakeSubsystem)
    //                             .withTimeout(1.0))));

    // Logger.recordOutput("Trajectories/starttoj",
    // startToJTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    //     "Trajectories/jtopickup", jToPickupTrajectory.getRawTrajectory().getPoses());
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

  private AutoRoutine createRoutine(
      AutoFactory factory,
      SwerveDrive swerve,
      VisionSubsystem visionSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      Source source,
      ReefBranch initialBranch,
      ReefBranch... cyclingBranches) {
    var routine = factory.newRoutine("Autogenerated Routine");

    var reefToSource = new AutoTrajectory[cyclingBranches.length];
    var sourceToReef = new AutoTrajectory[cyclingBranches.length];
    reefToSource[0] = getTrajectory(routine, initialBranch, source);
    sourceToReef[0] = getTrajectory(routine, source, cyclingBranches[0]);
    for (int i = 1; i < cyclingBranches.length; i++) {
      reefToSource[i] = getTrajectory(routine, cyclingBranches[i - 1], source);
      sourceToReef[i] = getTrajectory(routine, source, cyclingBranches[i]);
    }
    var nextCycleSpwnCmd = new Command[sourceToReef.length];
    for (int i = 0; i < sourceToReef.length - 1; i++) {
      nextCycleSpwnCmd[i] = reefToSource[i + 1].spawnCmd();
    }
    nextCycleSpwnCmd[sourceToReef.length - 1] =
        new ScheduleCommand(
            getTrajectory(routine, cyclingBranches[cyclingBranches.length - 1], source)
                .cmd()
                .andThen(sourcePathfinding.goTo(source)));

    routine
        .active()
        .onTrue(
            sequence(
                    waitUntil(swerve::nearingTargetPose),
                    elevatorSubsystem.setElevatorPosition(swerve::atTargetPoseAuto))
                .deadlineFor(reefPathfinding.goTo(initialBranch))
                .withName("ScoreAt" + initialBranch.name())
                .andThen(reefToSource[0].spawnCmd())
                .withName("StartTo" + initialBranch.name()));

    for (int i = 0; i < cyclingBranches.length; i++) {
      reefToSource[i]
          .done()
          .onTrue(
              waitUntil(() -> coralIntakeSubsystem::hasControl)
                  // .withTimeout(.5) // ONLY RUN IN SIM
                  .deadlineFor(sourcePathfinding.goTo(source))
                  .andThen(sourceToReef[i].spawnCmd())
                  .withName("Source" + source.name()));

      sourceToReef[i]
          .atTimeBeforeEnd(.5)
          .onTrue(
              elevatorSubsystem
                  .setElevatorPosition(swerve::atTargetPoseAuto)
                  .deadlineFor(
                      waitUntil(sourceToReef[i].done())
                          .andThen(reefPathfinding.goTo(cyclingBranches[i]).asProxy()))
                  .andThen(nextCycleSpwnCmd[i])
                  .withName("ScoreAt" + cyclingBranches[i].name()));
    }

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

  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);
  private final Alert loadedAutoAlert = new Alert("", Alert.AlertType.kInfo);

  public void update() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      var selected = chooser.get();
      if (selected.equals(selectedCommandName) && selectedOnRed == AllianceFlipper.isRed()) {
        return;
      }
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get().withName(selectedCommandName);
      selectedOnRed = AllianceFlipper.isRed();
      loadedAutoAlert.setText("Loaded Auto: " + selectedCommandName);
      loadedAutoAlert.set(true);
    }
  }

  public void clear() {
    selectedCommandName = NONE_NAME;
    selectedCommand = Commands.none();
    selectedOnRed = false;
  }

  public Command getSelectedCommand() {
    return selectedCommand;
  }

  private void addRoutine(String name, Supplier<AutoRoutine> generator) {
    chooser.addOption(name, name);
    routines.put(name, () -> generator.get().cmd());
  }

  private enum Source {
    L,
    R
  }

  private AutoTrajectory getTrajectory(AutoRoutine routine, ReefBranch reefBranch, Source source) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 0);
  }

  private AutoTrajectory getTrajectory(AutoRoutine routine, Source source, ReefBranch reefBranch) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 1);
  }
}
