package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.autodrive.RepulsorReef;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.FollowSwerveSampleCommand;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.sim.SimWorld;
import frc.robot.subsystems.algaePivot.AlgaePivotInterface;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.algaePivot.PhysicalAlgaePivot;
import frc.robot.subsystems.algaePivot.SimulatedAlgaePivot;
import frc.robot.subsystems.climbPivot.ClimbPivot;
import frc.robot.subsystems.climbPivot.ClimbPivotInterface;
import frc.robot.subsystems.climbPivot.PhysicalClimbPivot;
import frc.robot.subsystems.climbPivot.SimulatedClimbPivot;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeInterface;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem.IntakeState;
import frc.robot.subsystems.coralIntake.PhysicalCoralIntake;
import frc.robot.subsystems.coralIntake.SimulatedCoralntake;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorInterface;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.PhysicalElevator;
import frc.robot.subsystems.elevator.SimulatedElevator;
import frc.robot.subsystems.funnelPivot.FunnelPivotInterface;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;
import frc.robot.subsystems.funnelPivot.PhysicalFunnelPivot;
import frc.robot.subsystems.funnelPivot.SimulatedFunnelPivot;
import frc.robot.subsystems.leds.LEDConstants.LEDProcess;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.gyro.PhysicalGyro;
import frc.robot.subsystems.swerve.gyro.SimulatedGyro;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.swerve.module.PhysicalModule;
import frc.robot.subsystems.swerve.module.SimulatedModule;
import frc.robot.subsystems.vision.PhysicalVision;
import frc.robot.subsystems.vision.SimulatedVision;
import frc.robot.subsystems.vision.VisionInterface;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final Joystick buttonBoard = new Joystick(2);

  private VisionSubsystem visionSubsystem;
  private SwerveDrive swerveDrive;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private AlgaePivotSubsystem algaePivotSubsystem;
  private ClimbPivot climbPivotSubsystem;
  private LEDSubsystem ledSubsystem;

  private SimWorld simWorld;

  private AutoFactory autoFactory;
  private AutoChooser autoChooser;
  private Autos autos;

  private boolean isElevatorZeroed;

  public Robot() {
    checkGit();
    setupLogging();
    setupSubsystems();
    // setupAuto();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority and real time to improve loop timing
    Threads.setCurrentThreadPriority(true, HardwareConstants.HIGH_THREAD_PRIORITY);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority without real time
    Threads.setCurrentThreadPriority(false, HardwareConstants.LOW_THREAD_PRIORITY);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // if (isElevatorZeroed == false) {
    //   Commands.sequence(
    //       Commands.run(() -> elevatorSubsystem.setVolts(-2.0)).withTimeout(0.5),
    //       new InstantCommand(() -> elevatorSubsystem.resetPosition(0.0)),
    //       new InstantCommand(() -> elevatorSubsystem.setVolts(0.0)),
    //       new InstantCommand(() -> elevatorSubsystem.enableLimits(true, true)));
    // }
    swerveDrive.resetEstimatedPose(
        new Pose2d(
            swerveDrive.getEstimatedPose().getX(),
            swerveDrive.getEstimatedPose().getY(),
            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())));
    visionSubsystem.setOdometryInfo(
        swerveDrive.getOdometryRotation2d().getDegrees(), 0, swerveDrive.getEstimatedPose());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // if (isElevatorZeroed == false) {
    //   Commands.sequence(
    //       Commands.run(() -> elevatorSubsystem.setVolts(-2.0)).withTimeout(0.5),
    //       new InstantCommand(() -> elevatorSubsystem.resetPosition(0.0)),
    //       new InstantCommand(() -> elevatorSubsystem.setVolts(0.0)),
    //       new InstantCommand(() -> elevatorSubsystem.enableLimits(true, true)));
    // }
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDriverController();
    configureOperatorController();
  }

  private void configureDriverController() {
    // Driver Left Stick
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[1],
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[0]
        };

    // DRIVER BUTTONS
    Command driveCommand =
        new DriveCommand(
            swerveDrive,
            visionSubsystem,
            // Translation in the X direction
            driverLeftStick[0],
            // Translation in the Y direction
            driverLeftStick[1],
            // Rotation
            () -> JoystickUtil.modifyAxis(driverController::getRightX, 3),
            // Robot relative
            () -> !driverController.rightBumper().getAsBoolean(),
            // Rotation speed
            () -> driverController.leftBumper().getAsBoolean());
    swerveDrive.setDefaultCommand(driveCommand);

    // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverController
        .povRight()
        .onTrue(
            new InstantCommand(
                () ->
                    swerveDrive.resetEstimatedPose(
                        new Pose2d(
                            swerveDrive.getEstimatedPose().getX(),
                            swerveDrive.getEstimatedPose().getY(),
                            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())))));

    // Reset robot odometry based on the most recent vision pose measurement from april tags
    // This should be pressed when looking at an april tag
    driverController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())));

    // FieldConstants has all reef poses
    driverController
        .rightTrigger()
        .whileTrue(new RepulsorReef(swerveDrive, visionSubsystem, false));
    driverController.leftTrigger().whileTrue(new RepulsorReef(swerveDrive, visionSubsystem, true));
  }

  private void configureOperatorController() {
    JoystickButton intakeButton = new JoystickButton(buttonBoard, 0);
    JoystickButton outakeButton = new JoystickButton(buttonBoard, 1);
    JoystickButton scoreL1 = new JoystickButton(buttonBoard, 2);
    JoystickButton scoreL2 = new JoystickButton(buttonBoard, 3);
    JoystickButton scoreL3 = new JoystickButton(buttonBoard, 4);
    JoystickButton scoreL4 = new JoystickButton(buttonBoard, 5);
    // JoystickButton alignLeft = new JoystickButton(buttonBoard, 6);
    // JoystickButton alignRight = new JoystickButton(buttonBoard, 7);

    // operatorController.leftBumper().whileTrue(coralIntakeSubsystem.ejectCoral());
    // operatorController.leftTrigger().whileTrue(coralIntakeSubsystem.intakeCoral());
    operatorController
        .rightBumper()
        .whileTrue(elevatorSubsystem.manualElevator(() -> operatorController.getLeftY()));
    operatorController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.resetPosition(0.0), elevatorSubsystem));
    // operatorController
    //     .x()
    //     .whileTrue(funnelSubsystem.manualFunnel(() -> operatorController.getLeftY() * 0.6));
    // operatorController
    //     .y()
    //     .whileTrue(climbPivotSubsystem.manualPivotClimb(() -> operatorController.getLeftY()));
    // operatorController
    //     .a()
    //     .whileTrue(new RunCommand(() -> funnelSubsystem.setFunnelAngle(8.0), funnelSubsystem));
    /* Uncomment below to score the coral with controller, this scores with auto align
     * and I'm pretty sure it doesn't work well yet. (idk)
     *
     * We should probably make the ScoreL commands parallel but for now we're testing.
     */
    operatorController
        .a()
        .whileTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L1.getPosition()));

    operatorController
        .x()
        .whileTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L2.getPosition()));

    operatorController
        .b()
        .whileTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L3.getPosition()));

    operatorController
        .y()
        .whileTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()));

    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.runEnd(
                () -> coralIntakeSubsystem.setIntakeVelocity(CoralIntakeConstants.EJECT_SPEED),
                () ->
                    coralIntakeSubsystem.setIntakeVelocity(
                        CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                coralIntakeSubsystem));

    operatorController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
                new InstantCommand(() -> coralIntakeSubsystem.setIntakeState(IntakeState.IDLE)),
                Commands.runEnd(
                    () -> coralIntakeSubsystem.intakeCoral(),
                    () -> coralIntakeSubsystem.setIntakeState(IntakeState.STOPPED),
                    coralIntakeSubsystem)));

    operatorController
        .povUp()
        .whileTrue(climbPivotSubsystem.manualPivotClimb(() -> operatorController.getLeftY()));
    operatorController
        .povDown()
        .whileTrue(funnelSubsystem.manualFunnel(() -> operatorController.getLeftY()));

    operatorController
        .povRight()
        .whileTrue(
            Commands.runEnd(
                () -> coralIntakeSubsystem.setIntakeVelocity(2000),
                () -> coralIntakeSubsystem.setIntakeVelocity(0.0),
                coralIntakeSubsystem));

    // intakeButton.whileTrue(coralIntakeSubsystem.intakeCoral());
    // outakeButton.whileTrue(coralIntakeSubsystem.ejectCoral());
    // scoreL1.whileTrue(new ScoreL1(elevatorSubsystem, coralIntakeSubsystem));
    // scoreL2.whileTrue(new ScoreL2(elevatorSubsystem, coralIntakeSubsystem));
    // scoreL3.whileTrue(new ScoreL3(elevatorSubsystem, coralIntakeSubsystem));
    // scoreL4.whileTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));
  }

  private void checkGit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // This tells you if you have uncommitted changes in your project
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }

  private void setupLogging() {
    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Gets data from network tables
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        {
          // Replaying a log, set up replay source
          setUseTiming(false); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  @Override
  public void robotInit() {
    isElevatorZeroed = false;
  }

  private void setupSubsystems() {
    switch (Constants.getRobot()) {
      case COMP_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new PhysicalModule(SwerveConstants.compModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision());
        this.elevatorSubsystem = new ElevatorSubsystem(new PhysicalElevator());
        this.funnelSubsystem = new FunnelSubsystem(new PhysicalFunnelPivot());
        this.algaePivotSubsystem = new AlgaePivotSubsystem(new PhysicalAlgaePivot());
        this.climbPivotSubsystem = new ClimbPivot(new PhysicalClimbPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new PhysicalCoralIntake());
        this.simWorld = null;
      }
      case DEV_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new PhysicalModule(SwerveConstants.devModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.devModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.devModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.devModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision());
        this.elevatorSubsystem = new ElevatorSubsystem(new PhysicalElevator());
        this.funnelSubsystem = new FunnelSubsystem(new PhysicalFunnelPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new PhysicalCoralIntake());
        this.algaePivotSubsystem = new AlgaePivotSubsystem(new AlgaePivotInterface() {});
        this.climbPivotSubsystem = new ClimbPivot(new PhysicalClimbPivot());
        this.ledSubsystem = new LEDSubsystem();
        this.simWorld = null;
      }
      case SWERVE_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorInterface() {});
        this.funnelSubsystem = new FunnelSubsystem(new PhysicalFunnelPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeInterface() {});
        this.algaePivotSubsystem = new AlgaePivotSubsystem(new AlgaePivotInterface() {});
        this.climbPivotSubsystem = new ClimbPivot(new PhysicalClimbPivot());
        this.simWorld = null;
      }

      case SIM_ROBOT -> {
        /* Sim robot, instantiate physics sim IO implementations */
        this.simWorld = new SimWorld();
        this.swerveDrive =
            new SwerveDrive(
                new SimulatedGyro(simWorld.robot().getDriveTrain().getGyro()),
                new SimulatedModule(0, simWorld.robot().getDriveTrain()),
                new SimulatedModule(1, simWorld.robot().getDriveTrain()),
                new SimulatedModule(2, simWorld.robot().getDriveTrain()),
                new SimulatedModule(3, simWorld.robot().getDriveTrain()));

        this.visionSubsystem =
            new VisionSubsystem(new SimulatedVision(() -> simWorld.aprilTagSim()));
        this.swerveDrive.resetEstimatedPose(new Pose2d(10, 5, new Rotation2d()));
        this.elevatorSubsystem = new ElevatorSubsystem(new SimulatedElevator());
        this.funnelSubsystem = new FunnelSubsystem(new SimulatedFunnelPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new SimulatedCoralntake());
        this.algaePivotSubsystem = new AlgaePivotSubsystem(new SimulatedAlgaePivot());
        this.climbPivotSubsystem = new ClimbPivot(new SimulatedClimbPivot());
        this.ledSubsystem = new LEDSubsystem();
      }

      default -> {
        this.visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        this.swerveDrive =
            new SwerveDrive(
                new GyroInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {});
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorInterface() {});
        this.funnelSubsystem = new FunnelSubsystem(new FunnelPivotInterface() {});
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeInterface() {});
        this.algaePivotSubsystem = new AlgaePivotSubsystem(new AlgaePivotInterface() {});
        this.climbPivotSubsystem = new ClimbPivot(new ClimbPivotInterface() {});
        this.simWorld = null;
      }
    }
    ledSubsystem.setProcess(LEDProcess.DEFAULT);
  }

  private void setupAuto() {
    this.autoChooser = new AutoChooser();
    // this sets up the auto factory
    this.autoFactory =
        new AutoFactory(
            this.swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            this.swerveDrive
                ::resetEstimatedPose, // A function that resets the current robot pose to the
            (SwerveSample sample) -> {
              FollowSwerveSampleCommand followCommand =
                  new FollowSwerveSampleCommand(this.swerveDrive, this.visionSubsystem, sample);
              followCommand.execute();
              if (this.swerveDrive.isTrajectoryFinished(sample)) {
                followCommand.cancel();
              }
            }, // A function that follows a choreo trajectory
            AllianceFlipper.isRed(), // If alliance flipping should be enabled
            this.swerveDrive); // The drive subsystem

    this.autos =
        new Autos(
            autoFactory, swerveDrive, visionSubsystem, elevatorSubsystem, coralIntakeSubsystem);

    this.autoChooser.addRoutine("Example Auto", () -> this.autos.exampleAutoRoutine());
    this.autoChooser.addRoutine(
        AutoConstants.ONE_METER_AUTO_ROUTINE, () -> this.autos.oneMeterTestAutoRoutine());
    this.autoChooser.addRoutine(
        AutoConstants.ONE_CORAL_AUTO_ROUTINE, () -> this.autos.oneCoralAutoRoutine());
    // This updates the auto chooser
    SmartDashboard.putData("Auto Chooser", this.autoChooser);

    // This
    RobotModeTriggers.autonomous().whileTrue(this.autoChooser.selectedCommandScheduler());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (simWorld != null) {
      simWorld.update(() -> swerveDrive.getEstimatedPose());
    }
  }
}
