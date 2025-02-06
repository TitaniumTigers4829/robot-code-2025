package frc.robot.sim;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rectangle2d;
import frc.robot.extras.util.RuntimeLog;
import frc.robot.extras.util.mathutils.GeomUtil;
import frc.robot.sim.configs.SimDriveTrainConfig;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimGamePiece;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import frc.robot.sim.simMechanism.SimBattery;
import frc.robot.sim.simMechanism.SimDriveTrain;
import frc.robot.sim.simMechanism.SimIndexer;
import frc.robot.sim.simMechanism.SimIntake;
import frc.robot.sim.simMechanism.SimMechanism;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Represents a robot in the sim environment.
 *
 * <p>A robot is composed of:
 *
 * <ul>
 *   <li>A {@link SimDriveTrain} subclass that represents the robot's drivetrain.
 *   <li>A {@link SimIndexer} object that stores {@link SimGamePiece}s for the robot.
 *   <li>A {@link SimBattery} object that simulates the robot's battery.
 *   <li>A collection of {@link SimIntake} objects that represent the robot's intakes.
 *   <li>A collection of {@link SimMechanism} objects that represent the robot's mechanisms.
 * </ul>
 */
public class SimRobot<DrvTrn extends SimDriveTrain> {
  private final SimArena arena;
  private final DrvTrn driveTrain;
  // may move towards multi indexers soon
  private final SimIndexer gamePieceStorage;
  private final SimBattery battery = new SimBattery();
  private final ConcurrentLinkedQueue<SimIntake> intakes = new ConcurrentLinkedQueue<>();
  private final ConcurrentLinkedQueue<SimMechanism> mechanisms = new ConcurrentLinkedQueue<>();

  public <C extends SimDriveTrainConfig<DrvTrn, C>> SimRobot(
      SimArena arena, String name, C drivetrainConfig, int gamePieceStorageCapacity) {
    this.arena = arena;
    arena.robots.add(this);
    this.driveTrain = SimDriveTrain.createDriveTrain(this, drivetrainConfig);
    arena.withWorld(world -> world.addBody(driveTrain.chassis));
    this.gamePieceStorage = new SimIndexer(gamePieceStorageCapacity);
  }

  public void simTick() {
    driveTrain.simTick();
    // final Voltage batVolts = battery.getBatteryVoltage();
    for (var mechanism : mechanisms) {
      mechanism.update(Volts.of(12.0));
    }
  }

  /**
   * Returns an object that stores the timing data for the simulation.
   *
   * @return an object that stores the timing data for the simulation.
   */
  public SimEnvTiming timing() {
    return arena.timing;
  }

  /**
   * Creates a new intake for the robot and adds it to the simulation.
   *
   * @param boundingBox the bounding box of the intake in the simulation world, uses robot
   *     coordinates space.
   * @param acceptedGamePieceVariants the types of game pieces that the intake can accept, if no
   *     types are provided, the intake will accept all types of game pieces.
   * @return the newly created intake.
   */
  public SimIntake createIntake(
      Rectangle2d boundingBox, GamePieceVariant... acceptedGamePieceVariants) {
    var intake =
        new SimIntake(
            driveTrain,
            gamePieceStorage,
            GeomUtil.toDyn4jRectangle(boundingBox),
            acceptedGamePieceVariants);
    intakes.add(intake);
    arena.withWorld(world -> world.addContactListener(intake.getGamePieceContactListener()));
    RuntimeLog.debug("Created IntakeSimulation");
    return intake;
  }

  /**
   * Adds a {@link SimMechanism} to the robot.
   *
   * @param mechanism the {@link SimMechanism} to add to the robot.
   */
  public void addMechanism(SimMechanism mechanism) {
    mechanisms.add(mechanism);
    battery.addMechanism(mechanism);
    RuntimeLog.debug("Added SimMechanism to SimRobot");
  }

  public void removeMechanism(SimMechanism mechanism) {
    mechanisms.remove(mechanism);
    battery.removeMechanism(mechanism);
    RuntimeLog.debug("Removed SimMechanism from SimRobot");
  }

  /** Returns the {@link SimDriveTrain} object that represents the robot's drivetrain. */
  public DrvTrn getDriveTrain() {
    return driveTrain;
  }

  /**
   * Returns the {@link SimIndexer} object that is used to store {@link SimGamePiece}s for this
   * {@link SimRobot}.
   *
   * @return the {@link SimIndexer} object that is used to store {@link SimGamePiece}s.
   */
  public SimIndexer getGamePieceStorage() {
    return gamePieceStorage;
  }
}
