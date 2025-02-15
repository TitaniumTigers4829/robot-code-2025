package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.autodrive.AutoAlign;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.sim.SimWorld;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.algaePivot.PhysicalAlgaePivot;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorInterface;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.PhysicalElevator;
import frc.robot.subsystems.elevator.SimulatedElevator;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.PhysicalIntake;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.gyro.PhysicalGyro;
import frc.robot.subsystems.swerve.gyro.SimulatedGyro;
import frc.robot.subsystems.swerve.module.CompModule;
import frc.robot.subsystems.swerve.module.ModuleInterface;
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
  private final VisionSubsystem visionSubsystem;
  private final SwerveDrive swerveDrive;
  private final ElevatorSubsystem elevatorSubsystem;

  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new PhysicalIntake());
  private final AlgaePivotSubsystem algaePivotSubsystem =
      new AlgaePivotSubsystem(new PhysicalAlgaePivot());

  private final SimWorld simWorld;
  // private final ZeroElevator zeroElevator = new ZeroElevator();
  public AutoFactory autoFactory;
  public final AutoChooser autoChooser;
  public Autos autos;

  public Robot() {
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

    // Set up data receivers & replay source
    switch (Constants.ROBOT_TYPE) {
      case COMP_ROBOT, DEV_ROBOT, SWERVE_ROBOT:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Gets data from network tables
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM_ROBOT:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      default:
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
    switch (Constants.ROBOT_TYPE) {
      case COMP_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new CompModule(SwerveConstants.moduleConfigs[0]),
                new CompModule(SwerveConstants.moduleConfigs[1]),
                new CompModule(SwerveConstants.moduleConfigs[2]),
                new CompModule(SwerveConstants.moduleConfigs[3]));
        visionSubsystem = new VisionSubsystem(new PhysicalVision());
        elevatorSubsystem = new ElevatorSubsystem(new PhysicalElevator());
        simWorld = null;
      }
      case DEV_ROBOT -> {
        swerveDrive = new SwerveDrive(null, null, null, null, null);
        visionSubsystem = null;
        elevatorSubsystem = null;
        simWorld = null;
      }

      case SIM_ROBOT -> {
        /* Sim robot, instantiate physics sim IO implementations */
        simWorld = new SimWorld();
        swerveDrive =
            new SwerveDrive(
                new SimulatedGyro(simWorld.robot().getDriveTrain().getGyro()),
                new SimulatedModule(0, simWorld.robot().getDriveTrain()),
                new SimulatedModule(1, simWorld.robot().getDriveTrain()),
                new SimulatedModule(2, simWorld.robot().getDriveTrain()),
                new SimulatedModule(3, simWorld.robot().getDriveTrain()));

        visionSubsystem = new VisionSubsystem(new SimulatedVision(() -> simWorld.aprilTagSim()));
        swerveDrive.resetEstimatedPose(new Pose2d(10, 5, new Rotation2d()));
        elevatorSubsystem = new ElevatorSubsystem(new SimulatedElevator());
      }

      default -> {
        visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        swerveDrive =
            new SwerveDrive(
                new GyroInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {});
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorInterface() {});
        simWorld = null;
      }
    }
    autoChooser = new AutoChooser();
    // this sets up the auto factory
    // autoFactory =
    //     new AutoFactory(
    //         swerveDrive::getEstimatedPose, // A function that returns the current robot pose
    //         swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to
    // the
    //         (SwerveSample sample) -> {
    //           FollowSwerveSampleCommand followCommand =
    //               new FollowSwerveSampleCommand(swerveDrive, visionSubsystem, sample);
    //           followCommand.execute();
    //           if (swerveDrive.isTrajectoryFinished(sample)) {
    //             followCommand.cancel();
    //           }
    //         }, // A function that follows a choreo trajectory
    //         AllianceFlipper.isRed(), // If alliance flipping should be enabled
    //         swerveDrive); // The drive subsystem

    autos = new Autos(autoFactory);

    autoChooser.addRoutine("Example Auto", () -> autos.exampleAutoRoutine());
    autoChooser.addRoutine(
        AutoConstants.ONE_METER_AUTO_ROUTINE, () -> autos.oneMeterTestAutoRoutine());
    // This updates the auto chooser
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // This
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
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
    swerveDrive.resetEstimatedPose(
        new Pose2d(
            swerveDrive.getEstimatedPose().getX(),
            swerveDrive.getEstimatedPose().getY(),
            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    configureDriverController();
    configureOperatorController();

    swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose());
  }

  private void configureDriverController() {
    // Driver Left Stick
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[0],
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[1]
        };

    // DRIVER BUTTONS
    Command driveCommand =
        new DriveCommand(
            swerveDrive,
            visionSubsystem,
            // Translation in the Y direction
            driverLeftStick[1],
            // Translation in the X direction
            driverLeftStick[0],
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
        .a()
        .whileTrue(new AutoAlign(swerveDrive, visionSubsystem, FieldConstants.RED_REEF_ONE));
  }

  private void configureOperatorController() {
    // operatorController.b().whileTrue(Commands.none());
    // operatorController.y().whileTrue(Commands.none());
    // operatorController.x().whileTrue(Commands.none());
    operatorController
        .a()
        .whileTrue(new ManualElevator(elevatorSubsystem, () -> operatorController.getLeftY()));
    // Manual zero
    operatorController.leftBumper().whileTrue(new ZeroElevator(elevatorSubsystem));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Elevator Safety
    if (swerveDrive.getRoll() >= ElevatorConstants.MAX_ANGLE_X // gyro safety
        || swerveDrive.getRoll() <= ElevatorConstants.MIN_ANGLE_X
        || swerveDrive.getPitch() >= ElevatorConstants.MAX_ANGLE_Y
        || swerveDrive.getPitch()
            <= ElevatorConstants
                .MIN_ANGLE_Y) // TODO if robot is not in climbing state, then do this (AND logic)
    {
      // elevatorSubsystem.setDefaultCommand(new ZeroElevator(elevatorSubsystem)); // maybe works
      elevatorSubsystem.setElevatorPosition(0);
    }
  }

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
