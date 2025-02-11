package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sim.SimWorld;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.algaePivot.ManualAlgaePivot;
import frc.robot.commands.autodrive.AutoAlign;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.intake.Eject;
import frc.robot.commands.intake.Intake;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.sim.SimWorld;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.algaePivot.PhysicalAlgaePivot;
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

  private SimWorld simWorld = new SimWorld();
  private final VisionSubsystem visionSubsystem;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new PhysicalIntake());
  private final AlgaePivotSubsystem algaePivotSubsystem =
      new AlgaePivotSubsystem(new PhysicalAlgaePivot());

  public AutoFactory autoFactory;
  public final AutoChooser autoChooser;
  public Autos autos;

  private final SimWorld simWorld = new SimWorld();

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
      case COMP_ROBOT:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Gets data from network tables
        Logger.addDataReceiver(new NT4Publisher());
                /* Real robot, instantiate hardware IO implementations */

        swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new CompModule(SwerveConstants.moduleConfigs[0]),
                new CompModule(SwerveConstants.moduleConfigs[1]),
                new CompModule(SwerveConstants.moduleConfigs[2]),
                new CompModule(SwerveConstants.moduleConfigs[3]));
        visionSubsystem = new VisionSubsystem(new PhysicalVision());
        // simWorld = null;
        break;

      case SIM_ROBOT:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());

        /* Sim robot, instantiate physics sim IO implementations */
        // simWorld = new SimWorld();
        swerveDrive =
            new SwerveDrive(
                new SimulatedGyro(simWorld.robot().getDriveTrain().getGyro()),
                new SimulatedModule(0, simWorld.robot().getDriveTrain()),
                new SimulatedModule(1, simWorld.robot().getDriveTrain()),
                new SimulatedModule(2, simWorld.robot().getDriveTrain()),
                new SimulatedModule(3, simWorld.robot().getDriveTrain()));

        visionSubsystem = new VisionSubsystem(new SimulatedVision(() -> simWorld.aprilTagSim()));
        swerveDrive.resetEstimatedPose(new Pose2d(10, 5, new Rotation2d()));
        break;

      case DEV_ROBOT:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

        swerveDrive = new SwerveDrive(null, null, null, null, null);

        visionSubsystem = null;
        // simWorld = null;
        break;

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
        // simWorld = null;
      }
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    robotContainer = new RobotContainer();


    }


     autoChooser = new AutoChooser();

    // this sets up the auto factory
    autoFactory =
        new AutoFactory(
            swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to the
            swerveDrive::followTrajectory,  // Drive subsystem path follower function  
            AllianceFlipper.isRed(), // If alliance flipping should be enabled
            swerveDrive); // The drive subsystem

    autos = new Autos(autoFactory);

    autoChooser.addRoutine("Example Auto", () -> autos.exampleAutoRoutine());
    autoChooser.addRoutine(
        AutoConstants.ONE_METER_AUTO_ROUTINE, () -> autos.oneMeterTestAutoRoutine());
    // this updates the auto chooser
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }
  private void configureButtonBindings() {
    DoubleSupplier driverLeftStickX = driverController::getLeftX;
    DoubleSupplier driverLeftStickY = driverController::getLeftY;
    DoubleSupplier driverRightStickX = driverController::getRightX;
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () -> JoystickUtil.modifyAxisPolar(driverLeftStickX, driverLeftStickY, 3)[0],
          () -> JoystickUtil.modifyAxisPolar(driverLeftStickX, driverLeftStickY, 3)[1]
        };

    Trigger driverRightBumper = new Trigger(driverController.rightBumper());
    Trigger driverRightDirectionPad = new Trigger(driverController.pov(90));
    Trigger driverLeftDirectionPad = new Trigger(driverController.pov(270));

    driverController.a().whileTrue(new Intake(intakeSubsystem));
    driverController.b().whileTrue(new Eject(intakeSubsystem));
    driverController
        .x()
        .whileTrue(new ManualAlgaePivot(algaePivotSubsystem, operatorController::getLeftY));

    // // autodrive
    // Trigger driverAButton = new Trigger(driverController::getAButton);
    // lol whatever
    // // intake
    // Trigger operatorLeftTrigger = new Trigger(()->operatorController.getLeftTriggerAxis() > 0.2);
    // Trigger operatorLeftBumper = new Trigger(operatorController::getLeftBumper);
    // // amp and speaker
    // Trigger operatorBButton = new Trigger(operatorController::getBButton);
    // Trigger operatorRightBumper = new Trigger(operatorController::getRightBumper);
    // Trigger operatorRightTrigger = new Trigger(()->operatorController.getRightTriggerAxis() >
    // 0.2);
    // Trigger driverRightTrigger = new Trigger(()->driverController.getRightTriggerAxis() > 0.2);

    // // manual pivot and intake rollers
    // Trigger operatorAButton = new Trigger(operatorController::getAButton);
    // Trigger operatorXButton = new Trigger(operatorController::getXButton);
    // Trigger operatorYButton = new Trigger(operatorController::getYButton);
    // DoubleSupplier operatorRightStickY = operatorController::getRightY;
    // // unused
    // Trigger operatorUpDirectionPad = new Trigger(()->operatorController.getPOV() == 0);
    // Trigger operatorLeftDirectionPad = new Trigger(()->operatorController.getPOV() == 270);
    // Trigger operatorDownDirectionPad = new Trigger(()->operatorController.getPOV() == 180);
    // Trigger driverLeftTrigger = new Trigger(()->driverController.getLeftTriggerAxis() > 0.2);
    Trigger driverLeftBumper = new Trigger(driverController.leftBumper());

    // DRIVER BUTTONS
    Command driveCommand =
        new DriveCommand(
            swerveDrive,
            visionSubsystem,
            driverLeftStick[1],
            driverLeftStick[0],
            () -> JoystickUtil.modifyAxis(driverRightStickX, 3),
            () -> !driverRightBumper.getAsBoolean(),
            () -> driverLeftBumper.getAsBoolean());

    swerveDrive.setDefaultCommand(driveCommand);

    // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverRightDirectionPad.onTrue(
        new InstantCommand(
            () ->
                swerveDrive.resetEstimatedPose(
                    new Pose2d(
                        swerveDrive.getEstimatedPose().getX(),
                        swerveDrive.getEstimatedPose().getY(),
                        Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())))));

    // Reset robot odometry based on the most recent vision pose measurement from april tags
    // This should be pressed when looking at an april tag
    driverLeftDirectionPad.onTrue(
        new InstantCommand(
            () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())));

    // FieldConstants has all reef poses
    driverController
        .a()
        .whileTrue(new AutoAlign(swerveDrive, visionSubsystem, FieldConstants.RED_REEF_ONE));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // TODO: test this!
    // Switch thread to high priority to improve loop timing
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
         swerveDrive.resetEstimatedPose(
        new Pose2d(
            swerveDrive.getEstimatedPose().getX(),
            swerveDrive.getEstimatedPose().getY(),
            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())));

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SmartDashboard.putBoolean("Did Get Auto Run", false);
    SmartDashboard.putBoolean("Did Auto Routine Run", false);
    configureButtonBindings();

    swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose());

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
    if (Robot.isSimulation()) {
      simWorld.update(() -> swerveDrive.getEstimatedPose());
    }
  }
}
