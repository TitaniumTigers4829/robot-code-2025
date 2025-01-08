package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.simulation.field.SimulatedField;
import frc.robot.extras.simulation.mechanismSim.swerve.GyroSimulation;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveDriveSimulation;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation.WHEEL_GRIP;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.gyro.PhysicalGyro;
import frc.robot.subsystems.swerve.gyro.SimulatedGyro;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.swerve.module.PhysicalModule;
import frc.robot.subsystems.swerve.module.SimulatedModule;
import frc.robot.subsystems.vision.PhysicalVision;
// import frc.robot.subsystems.vision.SimulatedVision;
import frc.robot.subsystems.vision.VisionInterface;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController operatorController = new CommandXboxController(1);
  // private final Indexer indexer = new Indexer(new IndexerIOTalonFX());
  // private final Intake intake = new Intake(new IntakeIOTalonFX());
  // private final Pivot pivot = new Pivot(new PivotIOTalonFX());
  // private final Flywheel flywheel = new Flywheel(new FlywheelIOTalonFX());
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Simulation, we store them here in the robot container
  // private final SimulatedField simulatedArena;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final GyroSimulation gyroSimulation;

  // Subsystems
  // private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case COMPROBOT -> {
        /* Real robot, instantiate hardware IO implementations */

        /* Disable Simulations */
        // this.simulatedArena = null;
        this.gyroSimulation = null;
        this.swerveDriveSimulation = null;

        swerveDrive =
            new SwerveDrive(
                new PhysicalGyro(),
                new PhysicalModule(SwerveConstants.moduleConfigs[0]),
                new PhysicalModule(SwerveConstants.moduleConfigs[1]),
                new PhysicalModule(SwerveConstants.moduleConfigs[2]),
                new PhysicalModule(SwerveConstants.moduleConfigs[3]));
        visionSubsystem = new VisionSubsystem(new PhysicalVision());
      }

      case SIMROBOT -> {
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

    // swerveDrive.periodic();
    swerveDrive.setPose(startingPose);
  }

  public void teleopInit() {
    configureButtonBindings();
  }

  // public void intakeCallback(boolean hasNote) {
  //   if (hasNote) {
  //     driverController.setRumble(RumbleType.kBothRumble, 0.1);
  //     operatorController.setRumble(RumbleType.kBothRumble, 1);
  //   } else {
  //     driverController.setRumble(RumbleType.kBothRumble, 0);
  //     operatorController.setRumble(RumbleType.kBothRumble, 0);
  //   }
  // }
  private void configureButtonBindings() {
    DoubleSupplier driverLeftStickX = driverController::getLeftX;
    DoubleSupplier driverLeftStickY = driverController::getLeftY;
    DoubleSupplier driverRightStickX = driverController::getRightX;
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () -> JoystickUtil.modifyAxisPolar(driverLeftStickX, driverLeftStickY, 3)[0],
          () -> JoystickUtil.modifyAxisPolar(driverLeftStickX, driverLeftStickY, 3)[1]
        };

    DoubleSupplier operatorLeftStickX = operatorController::getLeftX;
    DoubleSupplier operatorRightStickY = operatorController::getRightY;

    Trigger driverRightBumper = new Trigger(driverController.rightBumper());
    Trigger driverRightDirectionPad = new Trigger(driverController.pov(90));
    Trigger driverDownDirectionPad = new Trigger(driverController.pov(180));
    Trigger driverLeftDirectionPad = new Trigger(driverController.pov(270));

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
    // Trigger driverBButton = new Trigger(driverController::getBButton);
    // Trigger driverYButton = new Trigger(driverController::getYButton);
    // DoubleSupplier operatorLeftStickY = operatorController::getLeftY;

    // //DRIVER BUTTONS

    // // driving

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

    // // shooterSubsystem.setDefaultCommand(new FlywheelSpinUpAuto(shooterSubsystem,
    // visionSubsystem));

    // driverLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // driverLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.3));
    // // Amp Lineup
    // driverAButton.whileTrue(new AutoAlignWithAmp(swerveDrive, visionSubsystem));
    // // Spinup for shoot
    // driverRightTrigger.whileTrue(new SpinUpForSpeaker(swerveDrive, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper,
    // ledSubsystem));

    // // driverLeftBumper.whileTrue(new ShootSpeaker(swerveDrive, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper,
    // ledSubsystem));
    // // driverRightTrigger.whileTrue(new ShootWhileMove(swerveDrive, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStick, driverYButton, ledSubsystem));

    // // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverRightDirectionPad.onTrue(
        new InstantCommand(
            () ->
                swerveDrive.setPose(
                    new Pose2d(
                        swerveDrive.getPose().getX(),
                        swerveDrive.getPose().getY(),
                        Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())))));
    driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> swerveDrive.setPose(swerveDriveSimulation.getSimulatedDriveTrainPose())));
    // // // Reset robot odometry based on vision pose measurement from april tags
    driverLeftDirectionPad.onTrue(
        new InstantCommand(() -> swerveDrive.setPose(visionSubsystem.getLastSeenPose())));
    // // driverLeftDpad.onTrue(new InstantCommand(() -> swerveDrive.resetOdometry(new
    // Pose2d(15.251774787902832, 5.573054313659668, Rotation2d.fromRadians(3.14159265)))));
    // // driverBButton.whileTrue(new ShootPass(swerveDrive, shooterSubsystem, pivotSubsystem,
    // visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));

    // // driverXButton.
    // driverBButton.whileTrue(new ShootPass(swerveDrive, shooterSubsystem, pivotSubsystem,
    // visionSubsystem, driverLeftStickY, operatorLeftStickY, driverYButton, ledSubsystem));
    // // driverDownDirectionPad.whileTrue(new IntakeFromShooter(shooterSubsystem,
    // intakeSubsystem));
    // // driverYButton.whileTrue(new ShootSpeaker(swerveDrive, shooterSubsystem, pivotSubsystem,
    // visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    // // OPERATOR BUTTONS

    // // speaker
    // operatorRightTrigger.whileTrue(new ShootSpeaker(swerveDrive, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper,
    // ledSubsystem));
    // // amp
    // operatorRightBumper.whileTrue(new ShootAmp(shooterSubsystem, pivotSubsystem, ledSubsystem,
    // operatorBButton));
    // // fender shot
    // operatorUpDirectionPad.whileTrue(new SubwooferShot(swerveDrive, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightStickX,
    // driverRightBumper, ledSubsystem));
    // // intake (aka SUCC_BUTTON)
    // operatorLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // operatorLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.2));
    // // outtake (aka UNSUCC_BUTTON)
    // operatorLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, true, ledSubsystem, this::intakeCallback));
    // // manual pivot (possible climb, unlikely)
    // operatorAButton.whileTrue(new ManualPivot(pivotSubsystem,
    // ()->modifyAxisCubed(operatorRightStickY)));
    // operatorDownDirectionPad.whileTrue(new ManualPivot(pivotSubsystem, ()->-0.2));
    // // manual rollers
    // operatorYButton.whileTrue(new ManualIntake(intakeSubsystem, true));
    // operatorXButton.whileTrue(new ManualIntake(intakeSubsystem, false));

    // operatorBButton.onTrue(new StopShooterAndIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem));
  }

  public Command getAutonomousCommand() {
    // Resets the pose factoring in the robot side
    // This is just a failsafe, pose should be reset at the beginning of auto
    swerveDrive.setPose(
        new Pose2d(
            swerveDrive.getPose().getX(),
            swerveDrive.getPose().getY(),
            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())));
    // return autoChooser.getSelected();
    // return new DriveForwardAndBack(swerveDrive);
    return null;
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
