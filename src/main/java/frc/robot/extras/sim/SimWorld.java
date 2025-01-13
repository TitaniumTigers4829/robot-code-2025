package frc.robot.extras.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.sim.configs.SimGyroConfig;
import frc.robot.extras.sim.configs.SimMechanismConfig;
import frc.robot.extras.sim.configs.SimSwerveConfig;
import frc.robot.extras.sim.configs.SimSwerveModuleConfig;
import frc.robot.extras.sim.configs.SimSwerveModuleConfig.WheelCof;
import frc.robot.extras.sim.utils.DCMotorExt;
import frc.robot.extras.sim.utils.GearRatio;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import static edu.wpi.first.units.Units.*;

public class SimWorld {

  private final SimArena arena;
  private final SimRobot<SimSwerve> simRobot;
    
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
          driveMotorCfg, steerMotorCfg, WheelCof.BLACK_NITRILE.cof, ModuleConstants.WHEEL_DIAMETER_METERS / 2.0);
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
      arena =
          new SimArena.FieldMap(Seconds.of(ConstValues.PERIODIC_TIME), 5);
      simRobot = new SimRobot<>(arena, "User", swerveConfig, 1);
    //   aprilTagSim = new VisionSystemSim("AprilTags");
    //   aprilTagSim.addAprilTags(FieldConstants.APRIL_TAG_FIELD);
    //   objectDetectionSim = new VisionSystemSim("ObjectDetection");
  }

  
  public SimArena arena() {
    return arena;
  }

  public SimRobot<SimSwerve> robot() {
    return simRobot;
  }

//   public VisionSystemSim aprilTagSim() {
//     return aprilTagSim;
//   }

//   public VisionSystemSim objectDetectionSim() {
//     return objectDetectionSim;
//   }

  public void update() {
    // if (isSimulation) {
    //   if (resetReceiver.hasData()) {
        // final var poses;
        robot().getDriveTrain().setChassisWorldPose(poses[poses.length - 1], true);
    //   }
      arena.simulationPeriodic();
      final Pose2d robotPose = simRobot.getDriveTrain().getChassisWorldPose();
    //   poseSender.send(new NamedPositions("SimRobot", robotPose));
    //   aprilTagSim.update(robotPose);
    //   objectDetectionSim.update(robotPose);
    //   objectDetectionSim.clearVisionTargets();
    //   final var objectTargets =
    //       arena
    //           .gamePieces()
    //           .map(gp -> new VisionTargetSim(gp.pose(), gpTargetModel))
    //           .toArray(VisionTargetSim[]::new);
    //   objectDetectionSim.addVisionTargets("gamepieces", objectTargets);
    // // }
  }
}
