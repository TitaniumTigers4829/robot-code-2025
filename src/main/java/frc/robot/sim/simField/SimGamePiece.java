package frc.robot.sim.simField;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.extras.math.forces.ProjectileUtil.ProjectileDynamics;
import frc.robot.extras.math.forces.Velocity3d;
import frc.robot.extras.math.mathutils.GeomUtil;
import frc.robot.sim.simMechanism.SimIntake;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collector;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;

/** A base class used for all game pieces in the simulation. */
public class SimGamePiece implements StructSerializable {
  public enum GamePieceState implements StructSerializable {
    LIMBO,
    ON_FIELD,
    IN_FLIGHT,
    HELD;
    public static final Struct<GamePieceState> struct =
        StructGenerator.genEnum(GamePieceState.class);
  }

  public record GamePieceVariant(
      String type,
      double height,
      double mass,
      DGeom shape,
      List<GamePieceTarget> targets,
      boolean autoPlaceOnGround,
      double landingDampening) {}

  public record GamePieceTarget(Rectangle2d area, Pair<Double, Double> heightRange) {
    public GamePieceTarget(Translation3d f, Translation3d s) {
      this(
          new Rectangle2d(f.toTranslation2d(), s.toTranslation2d()),
          new Pair<>(f.getZ(), s.getZ()));
    }

    public boolean isInside(Translation3d pos) {
      return area.contains(pos.toTranslation2d())
          && pos.getZ() >= heightRange.getFirst()
          && pos.getZ() <= heightRange.getSecond();
    }
  }

  protected sealed interface State {
    default void onEnter(SimGamePiece gp, SimArena arena) {}

    default void onExit(SimGamePiece gp, SimArena arena) {}

    default void tick(SimGamePiece gp, SimArena arena) {}

    Pose3d pose(SimGamePiece gp, SimArena arena);

    GamePieceState tag();
  }

  public static final class Limbo implements State {
    @Override
    public void onEnter(SimGamePiece gp, SimArena arena) {}

    @Override
    public Pose3d pose(SimGamePiece gp, SimArena arena) {
      return new Pose3d(-1, -1, -1, new Rotation3d());
    }

    @Override
    public GamePieceState tag() {
      return GamePieceState.LIMBO;
    }
  }

  public final class OnField implements State {
    private final GamePieceCollisionBody body;

    public OnField(GamePieceCollisionBody b) {
      this.body = b;
    }

    @Override
    public void onEnter(SimGamePiece gp, SimArena arena) {
      arena.gamePieces.add(gp);
      arena.getSpace().add(body.getGeom());
    }

    @Override
    public void onExit(SimGamePiece gp, SimArena arena) {
      arena.gamePieces.remove(gp);
      arena.getSpace().remove(body.getGeom());
    }

    @Override
    public void tick(SimGamePiece gp, SimArena arena) {}

    @Override
    public Pose3d pose(SimGamePiece gp, SimArena arena) {
      Translation3d t2d =
          GeomUtil.toWpilibPose(body.getBody().getPosition(), body.getBody().getQuaternion())
              .getTranslation();
      return new Pose3d(
          t2d.getX(),
          t2d.getY(),
          gp.variant.height / 2.0,
          new Rotation3d(0, 0, GeomUtil.toWpilibRotation(body.getBody().getQuaternion()).getZ()));
    }

    @Override
    public GamePieceState tag() {
      return GamePieceState.ON_FIELD;
    }
  }

  public static final class InFlight implements State {
    private final ProjectileDynamics dyn;
    private Pose3d pose;
    private Velocity3d vel;

    public InFlight(Pose3d p, Velocity3d v, ProjectileDynamics d) {
      this.pose = p;
      this.vel = v;
      this.dyn = d;
    }

    @Override
    public void tick(SimGamePiece gp, SimArena arena) {
      double dt = arena.timing.dt().in(Seconds);
      vel = dyn.calculate(dt, vel);
      Twist3d tw = new Twist3d(dt * vel.getVX(), dt * vel.getVY(), dt * vel.getVZ(), 0, 0, 0);
      pose = pose.exp(tw);
    }

    @Override
    public Pose3d pose(SimGamePiece gp, SimArena arena) {
      return pose;
    }

    @Override
    public GamePieceState tag() {
      return GamePieceState.IN_FLIGHT;
    }
  }

  public static final class Held implements State {
    private static final Pose3d DEFAULT = new Pose3d(0, 0, -1000, new Rotation3d());

    @Override
    public Pose3d pose(SimGamePiece gp, SimArena arena) {
      return DEFAULT;
    }

    @Override
    public GamePieceState tag() {
      return GamePieceState.HELD;
    }
  }

  // Collision and intake
  public static class GamePieceCollisionBody {
    private final DBody body;
    private final DGeom geom;

    public GamePieceCollisionBody(DBody b, DGeom g) {
      this.body = b;
      this.geom = g;
      geom.setBody(b);
    }

    public DGeom getGeom() {
      return geom;
    }

    public DBody getBody() {
      return body;
    }
  }

  protected final SimArena arena;
  protected final GamePieceVariant variant;
  protected State state = new Limbo();
  protected boolean userControlled = false;

  public SimGamePiece(GamePieceVariant v, SimArena a) {
    this.variant = v;
    this.arena = a;
  }

  private void transition(State ns) {
    state.onExit(this, arena);
    state = ns;
    state.onEnter(this, arena);
    RuntimeLog.debug("GP state -> " + state.tag());
  }

  public Pose3d pose() {
    return state.pose(this, arena);
  }

  public GamePieceState tag() {
    return state.tag();
  }

  public SimGamePiece userControlled() {
    userControlled = true;
    return this;
  }

  public boolean isUserControlled() {
    return userControlled;
  }

  public GamePieceVariant variant() {
    return variant;
  }

  public GamePieceState state() {
    return state.tag();
  }

  public void place(Translation2d pos) {
    if (!userControlled) {
      RuntimeLog.warn("No control");
      return;
    }
    // create body
    DBody db = OdeHelper.createBody(arena.getWorld());
    DGeom g = variant.shape;
    GamePieceCollisionBody gb = new GamePieceCollisionBody(db, g);
    transition(new OnField(gb));
    userControlled = false;
  }

  public void launch(Pose3d p, Velocity3d v, ProjectileDynamics d) {
    if (!userControlled) {
      RuntimeLog.warn("No control");
      return;
    }
    transition(new InFlight(p, v, d));
    userControlled = false;
  }

  public void intake() {
    if (userControlled) {
      transition(new Held());
      releaseControl();
    } else RuntimeLog.warn("No control");
  }

  public void delete() {
    transition(new Limbo());
    releaseControl();
  }

  public void releaseControl() {
    userControlled = false;
  }

  public void simulationSubTick() {
    state.tick(this, arena);
    if (state instanceof InFlight infl) {
      Pose3d p3 = infl.pose(this, arena);
      if (p3.getTranslation().getZ() < 0) place(new Translation2d(p3.getX(), p3.getY()));
    }
  }

  /** Called by arena each collision pair */
  public void nearCallback(DSpace space, DGeom a, DGeom b) {
    DGeom mine = (state instanceof OnField of) ? of.body.getGeom() : null;
    if (mine == null) return;
    DGeom other = a == mine ? b : (b == mine ? a : null);
    if (other == null) return;
    Object d = other.getData();
    if (d instanceof SimIntake si && userControlled == false) {
      // intake interaction
      si.startIntake(); // TODO: check
    }
  }

  public static Collector<SimGamePiece, ArrayList<Pose3d>, Pose3d[]> poseStreamCollector() {
    return new Collector<SimGamePiece, ArrayList<Pose3d>, Pose3d[]>() {
      @Override
      public Supplier<ArrayList<Pose3d>> supplier() {
        return () -> new ArrayList<>();
      }

      @Override
      public BiConsumer<ArrayList<Pose3d>, SimGamePiece> accumulator() {
        return (poses, gamePiece) -> poses.add(gamePiece.pose());
      }

      @Override
      public Set<Characteristics> characteristics() {
        return Set.of();
      }

      @Override
      public BinaryOperator<ArrayList<Pose3d>> combiner() {
        return (poses1, poses2) -> {
          poses1.addAll(poses2);
          return poses1;
        };
      }

      @Override
      public Function<ArrayList<Pose3d>, Pose3d[]> finisher() {
        return poses -> poses.toArray(new Pose3d[0]);
      }
    };
  }

  // public static final ShameGamePieceStruct struct = new ShameGamePieceStruct();

  // public static final class ShameGamePieceStruct implements Struct<SimGamePiece> {
  //     @Override
  //     public String getSchema() {
  //         return "Pose3d pose;char type[16];GamePieceState state";
  //     }

  //     @Override
  //     public int getSize() {
  //         return 16 + Pose3d.struct.getSize() + GamePieceState.struct.getSize();
  //     }

  //     @Override
  //     public Class<SimGamePiece> getTypeClass() {
  //         return SimGamePiece.class;
  //     }

  //     @Override
  //     public String getTypeName() {
  //         return "ShamGamePiece";
  //     }

  //     @Override
  //     public void pack(ByteBuffer bb, SimGamePiece value) {
  //         Pose3d.struct.pack(bb, value.pose());
  //         // bb.put(ProceduralStructGenerator.fixedSizeString(value.variant.type, 16));
  //         GamePieceState.struct.pack(bb, value.state());
  //     }

  //     @Override
  //     public SimGamePiece unpack(ByteBuffer bb) {
  //         throw new UnsupportedOperationException("Unpacking ShamGamePiece is not supported");
  //     }
  // }
}
