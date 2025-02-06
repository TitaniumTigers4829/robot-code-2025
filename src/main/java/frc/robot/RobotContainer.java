package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.algaePivot.ManualAlgaePivot;
import frc.robot.commands.autodrive.AutoAlign;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.intake.Eject;
import frc.robot.commands.intake.Intake;
import frc.robot.sim.SimWorld;
import frc.robot.extras.util.JoystickUtil;
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

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  private final SwerveDrive swerveDrive;

  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new PhysicalIntake());
  private final AlgaePivotSubsystem algaePivotSubsystem =
      new AlgaePivotSubsystem(new PhysicalAlgaePivot());

  private final SendableChooser<Command> autoChooser;

  private final SimWorld simWorld = new SimWorld();

  public RobotContainer() {
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Auto", null);

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
        // simWorld = null;
      }
      case DEV_ROBOT -> {
        swerveDrive = new SwerveDrive(null, null, null, null, null);

        visionSubsystem = null;
        // simWorld = null;
      }

      case SIM_ROBOT -> {
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
        // simWorld = null;
      }
    }
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

  public void simulationPeriodic() {
    if (Robot.isSimulation()) {
      simWorld.update(() -> swerveDrive.getEstimatedPose());
    }
  }
}
