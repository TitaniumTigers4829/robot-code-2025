package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.sim.SimArena;
import frc.robot.extras.sim.SimGyro;
import frc.robot.extras.sim.SimSwerve;
import frc.robot.extras.sim.SimArena.SimEnvTiming;
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
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  private final SwerveDrive swerveDrive;

  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Simulation, we store them here in the robot container
  private final SimArena simulatedArena;
  private final SimSwerve swerveDriveSimulation;
  private final SimGyro gyroSimulation;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Auto", null);

    switch (Constants.CURRENT_MODE) {
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
        SimEnvTiming timing = new SimEnvTiming(Time.of(0.02), 1, null);
        gyroSimulation = new SimGyro(, null);
        /* create simulations */
        /* create simulation for pigeon2 IMU (different IMUs have different measurement erros) */
        // this.gyroSimulation = GyroSimulation.createNavX2();
        /* create a swerve drive simulation */
        // this.swerveDriveSimulation =
        //     new SwerveDriveSimulation(
        //         SimulationConstants.ROBOT_MASS_KG,
        //         DriveConstants.TRACK_WIDTH,
        //         DriveConstants.WHEEL_BASE,
        //         DriveConstants.TRACK_WIDTH + .2,
        //         DriveConstants.WHEEL_BASE + .2,
        //         SwerveModuleSimulation.getModule(
        //             DCMotor.getFalcon500(1),
        //             DCMotor.getFalcon500(1),
        //             60,
        //             WHEEL_GRIP.TIRE_WHEEL,
        //             ModuleConstants.DRIVE_GEAR_RATIO),
        //         gyroSimulation,
        //         new Pose2d(3, 3, new Rotation2d()));
        // simulatedArena.addDriveTrainSimulation(swerveDriveSimulation);
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
        //     new VisionSubsystem(
        //         new SimulatedVision(() -> swerveDriveSimulation.getSimulatedDriveTrainPose()));

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
    driverController
        .x()
        .onTrue(
            new InstantCommand(
                () ->
                    swerveDrive.resetEstimatedPose(
                        swerveDriveSimulation.getSimulatedDriveTrainPose())));

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
    return autoChooser.getSelected();
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
