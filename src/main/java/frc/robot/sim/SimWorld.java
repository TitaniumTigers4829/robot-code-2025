package frc.robot.extras.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.sim.configs.SimGyroConfig;
import frc.robot.extras.sim.configs.SimMechanismConfig;
import frc.robot.extras.sim.configs.SimSwerveConfig;
import frc.robot.extras.sim.configs.SimSwerveModuleConfig;
import frc.robot.extras.sim.configs.SimSwerveModuleConfig.WheelCof;
import frc.robot.extras.sim.sim2025.Reefscape.ReefscapeSimArena;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.extras.util.GearRatio;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class SimWorld {

  private final SimArena arena;
  private final SimRobot<SimSwerve> simRobot;

  private final VisionSystemSim aprilTagSim;

  private final SimMechanismConfig driveMotorCfg =
      new SimMechanismConfig(new DCMotorExt(DCMotor.getKrakenX60Foc(1), 1))
          .withFriction(Volts.of(ModuleConstants.DRIVE_S), Volts.of(ModuleConstants.DRIVE_S * 0.8))
          .withGearRatio(GearRatio.reduction(ModuleConstants.DRIVE_GEAR_RATIO))
          .withNoise(0.00)
          .withRotorInertia(KilogramSquareMeters.of(0.003));
  private final SimMechanismConfig steerMotorCfg =
      new SimMechanismConfig(new DCMotorExt(DCMotor.getFalcon500Foc(1), 1))
          .withFriction(Volts.of(ModuleConstants.TURN_S), Volts.of(ModuleConstants.TURN_S * 0.8))
          .withGearRatio(GearRatio.reduction(ModuleConstants.TURN_GEAR_RATIO))
          .withNoise(0.00)
          .withRotorInertia(KilogramSquareMeters.of(0.02));
  private final SimSwerveModuleConfig moduleCfg =
      new SimSwerveModuleConfig(
          driveMotorCfg,
          steerMotorCfg,
          WheelCof.BLACK_NITRILE.cof,
          ModuleConstants.WHEEL_DIAMETER_METERS / 2.0);
  private final SimSwerveConfig swerveConfig =
      new SimSwerveConfig(
          60.0,
          6.0,
          Units.inchesToMeters(30.5),
          Units.inchesToMeters(30.5),
          DriveConstants.MODULE_TRANSLATIONS,
          moduleCfg,
          SimGyroConfig.ofNavX2());

  public SimWorld() {
    arena = new ReefscapeSimArena(Seconds.of(HardwareConstants.TIMEOUT_S), 5);
    simRobot = new SimRobot<>(arena, "User", swerveConfig, 1);

    aprilTagSim = new VisionSystemSim("AprilTags");
    aprilTagSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
  }

  /**
   * Returns the simulation arena.
   *
   * @return the simulation arena
   */
  public SimArena arena() {
    return arena;
  }

  /**
   * @return
   */
  public SimRobot<SimSwerve> robot() {
    return simRobot;
  }

  public VisionSystemSim aprilTagSim() {
    return aprilTagSim;
  }

  /** Updates the simulation. */
  public void update(Supplier<Pose2d> poseSupplier) {
    robot().getDriveTrain().setChassisWorldPose(poseSupplier.get(), true);
    arena().simulationPeriodic();

    final Pose2d robotPose = simRobot.getDriveTrain().getChassisWorldPose();
    aprilTagSim.update(robotPose);
    Logger.recordOutput("Odometry/ChassisPose", robot().getDriveTrain().getChassisWorldPose());
  }
}
