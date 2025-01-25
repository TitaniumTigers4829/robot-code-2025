package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.extras.simulation.field.SimulatedField;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
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
  private RobotContainer robotContainer;

  private ExampleSubsystem exampleSubsystem;
  private SwerveDrive swerveDrive;
  private AutoFactory autoFactory;
  public AutoChooser autoChooser;

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
    switch (Constants.CURRENT_MODE) {
      case COMP_ROBOT:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Gets data from network tables
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM_ROBOT:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case DEV_ROBOT:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    robotContainer = new RobotContainer();

    exampleSubsystem = new ExampleSubsystem();

    // this sets up the auto factory
    autoFactory =
        new AutoFactory(
            swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to the
            // provided Pose2d
            swerveDrive::followTrajectory, // The drive subsystem trajectory follower
            AllianceFlipper.isRed(), // If alliance flipping should be enabled
            swerveDrive); // The drive subsystem

    // this will put our autonomous chooser on the dashboard.
    autoChooser = new AutoChooser();
    autoChooser.addRoutine("Example routine", this::exampleAutoRoutine);
    SmartDashboard.putData(autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private AutoRoutine exampleAutoRoutine() {

    AutoRoutine routine = autoFactory.newRoutine("exampleAutoRoutine");

    AutoTrajectory startToETraj = routine.trajectory("startToE");
    AutoTrajectory eToPickupTraj = routine.trajectory("eToPickup");
    AutoTrajectory cToPickupTraj = routine.trajectory("cToPickup");
    AutoTrajectory pickupToCTraj = routine.trajectory("pickupToC");

    // reset odometry and start first trajectory
    routine.active().onTrue(Commands.sequence(startToETraj.resetOdometry(), startToETraj.cmd()));

    startToETraj
        .active()
        .onTrue(
            exampleSubsystem
                .exampleFunctionalCommand()); // TODO: replace with elevator to L4 command
    startToETraj
        .atTime("score")
        .onTrue(
            exampleSubsystem.exampleFunctionalCommand()); // TODO: replace with command for rollers
    startToETraj
        .done()
        .onTrue(
            eToPickupTraj
                .cmd()
                .alongWith(
                    exampleSubsystem
                        .exampleFunctionalCommand())); // TODO: replace with elevator to intake
    // command

    return routine;
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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    robotContainer.teleopInit();
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
    SimulatedField.getInstance().simulationPeriodic();
    robotContainer.updateFieldSimAndDisplay();
  }
}
