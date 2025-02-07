package frc.robot.extras.util.mathutils;

import static frc.robot.extras.util.mathutils.MeasureMath.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;

public class MassMath {
  public record PhysicsMass(Mass mass, MomentOfInertia moi) {
    public LinearAcceleration accelerationDueToForce(Force force) {
      return force.div(mass);
    }

    public AngularAcceleration accelerationDueToTorque(Torque torque) {
      return div(torque, moi);
    }

    public Pair<XY<LinearAcceleration>, AngularAcceleration> accelerationsDueToForce(
        XY<Force> forces, XY<Distance> forcePosition) {
      final LinearAcceleration xAccel = accelerationDueToForce(forces.x());
      final LinearAcceleration yAccel = accelerationDueToForce(forces.y());
      final AngularAcceleration omegaAccel =
          accelerationDueToTorque(forces.cross(forcePosition, Torque.class, MeasureMath::times));
      return new Pair<>(new XY<>(xAccel, yAccel), omegaAccel);
    }

    /**
     * Applies a force due to acceleration.
     * @param acceleration
     * @return
     */
    public Force forceDueToAcceleration(LinearAcceleration acceleration) {
      return times(acceleration, mass);
    }

    public Torque torqueDueToAcceleration(AngularAcceleration acceleration) {
      return times(acceleration, moi);
    }

    public Pair<XY<Force>, Torque> forcesDueToAcceleration(
        XY<LinearAcceleration> acceleration, XY<Distance> forcePosition) {
      final Force xForce = forceDueToAcceleration(acceleration.x());
      final Force yForce = forceDueToAcceleration(acceleration.y());
      final Torque omegaForce =
          times(forcePosition.x(), yForce).minus(times(forcePosition.y(), xForce));
      return new Pair<>(new XY<>(xForce, yForce), omegaForce);
    }

    public Pair<XY<Force>, Torque> forcesDueToOffsetForces(
        XY<Force> forces, XY<Distance> forcePosition) {
      final Force xForce = forces.x();
      final Force yForce = forces.y();
      final Torque omegaForce = forces.cross(forcePosition, Torque.class, MeasureMath::times);
      return new Pair<>(new XY<>(xForce, yForce), omegaForce);
    }
  }
}
