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
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

/**
 * Abstract simulation arena using ODE4j with a 2D public API.
 */
public abstract class SimArena {
  public record SimEnvTiming(Time period, int ticksPerPeriod, Time dt)
      implements StructSerializable {
    private SimEnvTiming(double robotPeriodSeconds, int subTicks) {
      this(
        Seconds.of(robotPeriodSeconds),
        subTicks,
        Seconds.of(robotPeriodSeconds / (double) subTicks)
      );
    }
    public static final Struct<SimEnvTiming> struct =
      StructGenerator.genRecord(SimEnvTiming.class);
  }

  public final ReentrantLock worldLock = new ReentrantLock();
  private static  DWorld odeWorld;
    private static DSpace odeSpace;
    public final Set<SimRobot<?>> robots      = ConcurrentHashMap.newKeySet();
    public final Set<SimGamePiece> gamePieces = ConcurrentHashMap.newKeySet();
    public final SimEnvTiming timing;
  
    // dispatch callback to all sim components:
    private final DNearCallback nearCb = new DNearCallback() {
      @Override
      public void call(Object data, DGeom g1, DGeom g2) {
        nearCallback((DSpace) data, g1, g2);
      }
    };
  
    protected SimArena(FieldMap obstaclesMap, double period, int ticksPerPeriod) {
      this.timing   = new SimEnvTiming(period, ticksPerPeriod);
      this.odeWorld = OdeHelper.createWorld();
      odeWorld.setGravity(0,0,0);
      this.odeSpace = OdeHelper.createSimpleSpace(null);
  
      // add static obstacles
      for (FrcBody obs : obstaclesMap.obstacles) {
        odeSpace.add(obs.getFirstGeom());
      }
    }
  
    /** Sim loop entry point; call from robot simulationPeriodic(). */
    public void simulationPeriodic() {
      competitionPeriodic();
      for (int i = 0; i < timing.ticksPerPeriod; i++) {
        robots.forEach(r -> r.simTick());
        gamePieces.forEach(SimGamePiece::simulationSubTick);
  
        worldLock.lock();
        try {
          // collision detection:
          odeSpace.collide(null, nearCb);
          // integrate bodies:
          odeWorld.step(timing.dt.in(Seconds));
        } finally {
          worldLock.unlock();
        }
      }
    }
  
    /** Dispatches collision events to registered sim elements. */
    private void nearCallback(DSpace space, DGeom g1, DGeom g2) {
      // e.g.:
      // for (SimIntake intake : allIntakes) intake.nearCallback(space, g1, g2);
      // for (SimGamePiece gp : gamePieces) gp.nearCallback(space, g1, g2);
    }
  
    // Abstract methods:
    protected abstract void competitionPeriodic();
    protected abstract void placeGamePiecesOnField();
  
    public SimGamePiece createGamePiece(GamePieceVariant v) {
      RuntimeLog.debug("Create GP "+v);
      SimGamePiece gp = new SimGamePiece(v, this);
      gamePieces.add(gp);
      return gp.userControlled();
    }
  
    public void resetFieldForAuto() {
      gamePieces.forEach(gp -> gp.delete());
      gamePieces.clear();
      placeGamePiecesOnField();
    }
  
    public Stream<SimGamePiece> gamePieces() {
      return gamePieces.stream();
    }
  
    // Expose ODE world/space:
    public static DWorld getWorld() { return odeWorld; }
  public static DSpace getSpace() { return odeSpace; }

  /** Field‐map builder for static obstacles. */
  public static class FieldMap {
    private final List<FrcBody> obstacles = new ArrayList<>();

    protected void addRectangularObstacle(double w, double h, Pose2d pose, DWorld world) {
      DGeom box = OdeHelper.createBox(getSpace(), w, h, 0.1);
      FrcBody fb = new FrcBody(world, box);
      fb.getBody().setPosition(pose.getX(), pose.getY(), 0);
      obstacles.add(fb);
    }
    private void addCustomObstacle(Translation2d a, Translation2d b, double thickness) {
      FrcBody obs = new FrcBody(getWorld(), OdeHelper.createCapsule(thickness, a.getDistance(b)));
      obs.getBody().setPosition((a.getX() + b.getX()) / 2, (a.getY() + b.getY()) / 2, 0);
      obstacles.add(obs);
    }
  }
}
