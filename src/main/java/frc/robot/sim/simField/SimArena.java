package frc.robot.sim.simField;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.extras.util.FrcBody;
import frc.robot.sim.SimRobot;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import frc.robot.sim.simMechanism.SimDriveTrain;
import frc.robot.sim.simMechanism.SimIntake;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Stream;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

/** Abstract simulation arena using ODE4j with a 2D public API. */
public abstract class SimArena {
  public record SimEnvTiming(Time period, int ticksPerPeriod, Time dt)
      implements StructSerializable {
    private SimEnvTiming(double robotPeriodSeconds, int subTicks) {
      this(
          Seconds.of(robotPeriodSeconds),
          subTicks,
          Seconds.of(robotPeriodSeconds / (double) subTicks));
    }

    public static final Struct<SimEnvTiming> struct = StructGenerator.genRecord(SimEnvTiming.class);
  }

  // Physics world & space
  private final DWorld odeWorld;
  private final DSpace odeSpace;

  // Concurrency
  public final ReentrantLock worldLock = new ReentrantLock();

  // Simulation objects
  public final Set<SimRobot<?>> robots = ConcurrentHashMap.newKeySet();
  public final Set<SimGamePiece> gamePieces = ConcurrentHashMap.newKeySet();
  private final List<SimDriveTrain> drivetrains = new ArrayList<>();
  private final List<SimIntake> intakes = new ArrayList<>();

  public final SimEnvTiming timing;

  // Single collision callback
  private final DNearCallback nearCb =
      new DNearCallback() {
        @Override
        public void call(Object data, DGeom g1, DGeom g2) {
          // Dispatch to intakes
          for (SimIntake intake : intakes) {
            intake.nearCallback((DSpace) data, g1, g2);
          }
          // Dispatch to game pieces
          for (SimGamePiece gp : gamePieces) {
            gp.nearCallback((DSpace) data, g1, g2);
          }
        }
      };

  protected SimArena(FieldMap obstaclesMap, double period, int ticksPerPeriod) {
    this.timing = new SimEnvTiming(period, ticksPerPeriod);
    this.odeWorld = OdeHelper.createWorld();
    odeWorld.setGravity(0, 0, 0);
    this.odeSpace = OdeHelper.createSimpleSpace(null);

    // Add static obstacles
    for (FrcBody obs : obstaclesMap.obstacles) {
      odeSpace.add(obs.getFirstGeom());
    }
  }

  /** Add a drivetrain’s chassis into the physics space. */
  public void addDriveTrain(SimDriveTrain drive) {
    drivetrains.add(drive);
    odeSpace.add(drive.getChassisBody().getFirstGeom());
  }

  /** Add an intake into the physics space and enable its collisions. */
  public void addIntake(SimIntake intake) {
    intakes.add(intake);
    odeSpace.add(intake.getIntakeGeom());
  }

  /** Main sim loop; call from your robot’s simulationPeriodic(). */
  public void simulationPeriodic() {
    competitionPeriodic();
    for (int i = 0; i < timing.ticksPerPeriod; i++) {
      robots.forEach(SimRobot::simTick);
      gamePieces.forEach(SimGamePiece::simulationSubTick);

      worldLock.lock();
      try {
        odeSpace.collide(null, nearCb);
        odeWorld.step(timing.dt.in(Seconds));
      } finally {
        worldLock.unlock();
      }
    }
  }

  /** Season‐specific logic each simulationPeriodic(). */
  protected abstract void competitionPeriodic();

  /** Place initial game pieces on the field. */
  protected abstract void placeGamePiecesOnField();

  /** Create and register a game piece in the arena. */
  public SimGamePiece createGamePiece(GamePieceVariant v) {
    RuntimeLog.debug("Create GP " + v);
    SimGamePiece gp = new SimGamePiece(v, this);
    gamePieces.add(gp);
    return gp.userControlled();
  }

  /** Reset field for autonomous mode. */
  public void resetFieldForAuto() {
    gamePieces.forEach(SimGamePiece::delete);
    gamePieces.clear();
    placeGamePiecesOnField();
  }

  /** Stream of all game pieces. */
  public Stream<SimGamePiece> gamePieces() {
    return gamePieces.stream();
  }

  /** Access to the physics world. */
  public DWorld getWorld() {
    return odeWorld;
  }

  /** Access to the collision space. */
  public DSpace getSpace() {
    return odeSpace;
  }

  // --------------------------------------------------------------------------------
  // Non-static FieldMap inner class to build obstacles tied to this arena instance
  // --------------------------------------------------------------------------------
  // TODO: consider static nested class
  public static class FieldMap {
    private final List<FrcBody> obstacles = new ArrayList<>();

    /** Add a thin rectangular obstacle (flat box) at the given 2D pose. */
    protected void addRectangularObstacle(
        DSpace odeSpace, DWorld odeWorld, double w, double h, Pose2d pose) {
      DGeom box = OdeHelper.createBox(odeSpace, w, h, 0.1);
      FrcBody fb = new FrcBody(odeWorld, box);
      fb.getBody().setPosition(pose.getX(), pose.getY(), 0);
      obstacles.add(fb);
    }

    /** Add a capsule’s edge between two 2D points with given thickness. */
    protected void addCustomObstacle(
        DSpace odeSpace, DWorld odeWorld, Translation2d a, Translation2d b, double thickness) {
      double length = a.getDistance(b);
      DGeom cap = OdeHelper.createCapsule(odeSpace, thickness, length);
      FrcBody fb = new FrcBody(odeWorld, cap);
      double cx = (a.getX() + b.getX()) / 2.0;
      double cy = (a.getY() + b.getY()) / 2.0;
      fb.getBody().setPosition(cx, cy, 0);
      obstacles.add(fb);
    }
  }
}
