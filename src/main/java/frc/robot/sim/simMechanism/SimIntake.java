package frc.robot.sim.simMechanism;

import frc.robot.sim.simField.SimGamePiece;
import frc.robot.sim.simField.SimGamePiece.GamePieceCollisionBody;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Function;

import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;

/**
 * Intake simulation using ODE4j 3D. Attach a geom to the drivetrain chassis
 * and listen for contacts to collect game pieces.
 */
public class SimIntake {
  private final DBody      chassisBody;
  private final DGeom      intakeGeom;
  private final DSpace     space;
  private final SimIndexer storage;
  private final List<GamePieceVariant> variants;
  private final AtomicBoolean running = new AtomicBoolean(false);

  /**
   * @param driveTrain   the drivetrain simulation (to attach intake to)
   * @param storage      where collected pieces go
   * @param shapeBuilder function: given DSpace, returns a configured DGeom
   * @param variants     which game piece variants to collect
   */
  public SimIntake(
      SimDriveTrain driveTrain,
      SimIndexer storage,
      Function<DSpace, DGeom> shapeBuilder,
      GamePieceVariant... variants) {
    this.chassisBody = driveTrain.getChassisBody();
    this.space       = driveTrain.getOdeWorld().getSpace();
    this.storage     = storage;
    this.variants    = List.of(variants);

    // build and attach geometry
    this.intakeGeom = shapeBuilder.apply(space);
    this.intakeGeom.setBody(chassisBody);
  }

  /** Enable intake: add geom to space and start collecting. */
  public void startIntake() {
    if (running.getAndSet(true)) return;
    space.add(intakeGeom);
  }

  /** Disable intake: remove geom and stop collecting. */
  public void stopIntake() {
    if (!running.getAndSet(false)) return;
    space.remove(intakeGeom);
  }

  /** ODE4j “near” callback—call this each physics step to check contacts. */
  public void nearCallback(DSpace space, DGeom g1, DGeom g2) {
    if (!running.get()) return;
    DGeom other = (g1 == intakeGeom) ? g2 : (g2 == intakeGeom ? g1 : null);
    if (other == null) return;

    Object data = other.getData();
    if (!(data instanceof GamePieceCollisionBody gp)) return;
    if (!variants.contains(gp.gp.variant())) return;

    // collect and remove piece
    storage.insertGamePiece(gp.gp);
    gp.gp.delete(); // TODO: check if this is correct
  }
}
