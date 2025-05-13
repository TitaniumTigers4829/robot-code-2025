package frc.robot.sim.simMechanism;

import frc.robot.sim.simField.SimGamePiece;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Function;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;

/** Intake simulation using ODE4j 3D. */
public class SimIntake {
  private final DBody chassisBody;
  private final DGeom intakeGeom;
  private final DSpace space;
  private final SimIndexer storage;
  private final List<GamePieceVariant> variants;
  private final AtomicBoolean running = new AtomicBoolean(false);

  /**
   * @param driveTrain the drivetrain (to attach intake to)
   * @param storage where collected pieces go
   * @param shapeBuilder given a space, returns a configured geom
   * @param variants which piece variants to collect
   */
  public SimIntake(
      SimDriveTrain driveTrain,
      SimIndexer storage,
      Function<DSpace, DGeom> shapeBuilder,
      GamePieceVariant... variants) {
    this.chassisBody = driveTrain.getChassisBody();
    this.space = driveTrain.getOdeWorld().getSpace();
    this.storage = storage;
    this.variants = List.of(variants);

    this.intakeGeom = shapeBuilder.apply(space);
    this.intakeGeom.setBody(chassisBody);
  }

  /** Turn intake on (adds geom into space). */
  public void startIntake() {
    if (running.getAndSet(true)) return;
    space.add(intakeGeom);
  }

  /** Turn intake off (removes geom). */
  public void stopIntake() {
    if (!running.getAndSet(false)) return;
    space.remove(intakeGeom);
  }

  /**
   * Called by arena’s DNearCallback every sub-tick. If intake is running and a piece collides,
   * attempt to store it.
   */
  public void nearCallback(DSpace space, DGeom g1, DGeom g2) {
    if (!running.get()) return;
    DGeom other = (g1 == intakeGeom) ? g2 : (g2 == intakeGeom ? g1 : null);
    if (other == null) return;

    Object data = other.getData();
    if (!(data instanceof SimGamePiece)) return;
    SimGamePiece piece = (SimGamePiece) data;
    if (!variants.isEmpty() && !variants.contains(piece.variant())) return;

    // only collect if piece is ON_FIELD
    if (piece.state() == SimGamePiece.GamePieceState.ON_FIELD) {
      if (storage.insertGamePiece(piece)) {
        // remove from field
        piece.delete();
      }
    }
  }

  /** Expose geom so arena can add/remove it. */
  public DGeom getIntakeGeom() {
    return intakeGeom;
  }

  /** Check if intake is currently active. */
  public boolean isRunning() {
    return running.get();
  }
}
