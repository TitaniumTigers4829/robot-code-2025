package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.sim.SimWorld;
import frc.robot.extras.util.JoystickUtil;
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

  private final SendableChooser<Command> autoChooser;

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
      }
      case DEV_ROBOT -> {
        swerveDrive = new SwerveDrive(null, null, null, null, null);

        visionSubsystem = null;
      }

      case SIM_ROBOT -> {
        /* Sim robot, instantiate physics sim IO implementations */
        SimWorld simWorld = new SimWorld();
        swerveDrive =
            new SwerveDrive(
                new SimulatedGyro(simWorld.robot().getDriveTrain().getGyro()),
                new SimulatedModule(0, simWorld.robot().getDriveTrain()),
                new SimulatedModule(1, simWorld.robot().getDriveTrain()),
                new SimulatedModule(2, simWorld.robot().getDriveTrain()),
                new SimulatedModule(3, simWorld.robot().getDriveTrain()));

        visionSubsystem =
            new VisionSubsystem(
                new SimulatedVision(() -> simWorld.robot().getDriveTrain().getChassisWorldPose()));
        simWorld
            .robot()
            .getDriveTrain()
            .setChassisWorldPose(new Pose2d(2, 2, Rotation2d.kZero), false);
        simWorld.arena().resetFieldForAuto();
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
}
