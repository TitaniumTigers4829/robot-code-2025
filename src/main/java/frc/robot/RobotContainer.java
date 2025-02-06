package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.FollowChoreoTrajectory;
import frc.robot.extras.simulation.field.SimulatedField;
import frc.robot.extras.simulation.mechanismSim.swerve.GyroSimulation;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveDriveSimulation;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation.WHEEL_GRIP;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.gyro.PhysicalGyro;
import frc.robot.subsystems.swerve.gyro.SimulatedGyro;
import frc.robot.subsystems.swerve.module.CompModule;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.swerve.module.SimulatedModule;
import frc.robot.subsystems.vision.PhysicalVision;
import frc.robot.subsystems.vision.VisionInterface;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.security.Key;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Simulation, we store them here in the robot container
  // private final SimulatedField simulatedArena;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final GyroSimulation gyroSimulation;

  public AutoFactory autoFactory;
  public AutoChooser autoChooser;
  public Autos autos;

  public RobotContainer() {

    switch (Constants.ROBOT_TYPE) {
      case COMP_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        /* Disable Simulations */
        // this.simulatedArena = null;
        this.gyroSimulation = null;
        this.swerveDriveSimulation = null;

        swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new CompModule(SwerveConstants.moduleConfigs[0]),
                new CompModule(SwerveConstants.moduleConfigs[1]),
                new CompModule(SwerveConstants.moduleConfigs[2]),
                new CompModule(SwerveConstants.moduleConfigs[3]));
        visionSubsystem = new VisionSubsystem(new PhysicalVision());
      }
      case DEV_ROBOT -> {
        swerveDrive = new SwerveDrive(null, null, null, null, null);
        gyroSimulation = null;
        swerveDriveSimulation = null;
        visionSubsystem = null;
      }

      case SIM_ROBOT -> {
        /* Sim robot, instantiate physics sim IO implementations */

        /* create simulations */
        /* create simulation for pigeon2 IMU (different IMUs have different measurement erros) */
        this.gyroSimulation = GyroSimulation.createNavX2();
        /* create a swerve drive simulation */
        this.swerveDriveSimulation =
            new SwerveDriveSimulation(
                SimulationConstants.ROBOT_MASS_KG,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.WHEEL_BASE,
                DriveConstants.TRACK_WIDTH + .2,
                DriveConstants.WHEEL_BASE + .2,
                SwerveModuleSimulation.getModule(
                    DCMotor.getFalcon500(1),
                    DCMotor.getFalcon500(1),
                    60,
                    WHEEL_GRIP.TIRE_WHEEL,
                    ModuleConstants.DRIVE_GEAR_RATIO),
                gyroSimulation,
                new Pose2d(3, 3, new Rotation2d()));
        SimulatedField.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        swerveDrive =
            new SwerveDrive(
                new SimulatedGyro(
                    gyroSimulation), // SimulatedGyro is a wrapper around gyro simulation, that
                // reads
                // the simulation result
                /* SimulatedModule are edited such that they also wraps around module simulations */
                new SimulatedModule(swerveDriveSimulation.getModules()[0]),
                new SimulatedModule(swerveDriveSimulation.getModules()[1]),
                new SimulatedModule(swerveDriveSimulation.getModules()[2]),
                new SimulatedModule(swerveDriveSimulation.getModules()[3]));

        visionSubsystem = null;

        SimulatedField.getInstance().resetFieldForAuto();
        resetFieldAndOdometryForAuto(
            new Pose2d(1.3980597257614136, 5.493067741394043, Rotation2d.fromRadians(3.1415)));
      }

      default -> {
        visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        this.gyroSimulation = null;
        this.swerveDriveSimulation = null;
        // this.simulatedArena = null;
        swerveDrive =
            new SwerveDrive(
                new GyroInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {});
      }
    }

    this.autoChooser = autoChooser;
    // this sets up the auto factory
    autoFactory =
        new AutoFactory(
            swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to the
            // provided Pose2d
            (SwerveSample sample) -> {
              FollowChoreoTrajectory command =
                  new FollowChoreoTrajectory(swerveDrive, visionSubsystem, sample);
              command.execute();
            }, // The drive subsystem trajectory follower
            AllianceFlipper.isRed(), // If alliance flipping should be enabled
            swerveDrive); // The drive subsystem

    autos = new Autos(autoFactory, swerveDrive);
    // this adds an auto routine to the auto chooser
    autoChooser.addRoutine("Example routine", () -> autos.exampleAutoRoutine());
    // this updates the auto chooser
    SmartDashboard.putData(autoChooser);
  }

  private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
    final Pose2d startingPose = robotStartingPoseAtBlueAlliance;
    
    if (swerveDriveSimulation != null) {
      swerveDriveSimulation.setSimulationWorldPose(startingPose);
      SimulatedField.getInstance().resetFieldForAuto();
      updateFieldSimAndDisplay();
    }

    swerveDrive.resetEstimatedPose(startingPose);
  }

  public void teleopInit() {
    configureButtonBindings();
    swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose());
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
    // driverController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 swerveDrive.resetEstimatedPose(
    //                     swerveDriveSimulation.getSimulatedDriveTrainPose())));

    // Reset robot odometry based on the most recent vision pose measurement from april tags
    // This should be pressed when looking at an april tag
    driverLeftDirectionPad.onTrue(
        new InstantCommand(
            () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())));
  }

  public Command getAutonomousCommand() {
    // Resets the pose factoring in the robot side
    // This is just a failsafe, pose should be reset at the beginning of auto
    swerveDrive.resetEstimatedPose(
        new Pose2d(
            swerveDrive.getEstimatedPose().getX(),
            swerveDrive.getEstimatedPose().getY(),
            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())));
    return autoChooser.selectedCommand();
  }

  public void updateFieldSimAndDisplay() {
    if (swerveDriveSimulation == null) return;
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedField.getInstance().getGamePiecesByType("Note").toArray(Pose3d[]::new));
        


      
  }
}
