package frc.robot.extras.sim.utils;

import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.dyn4j.dynamics.Body;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.ProceduralStructGenerator;
import sham.utils.geometry.Velocity2d;
import sham.utils.mathutils.GeometryConvertor;

public class FrcBody extends Body {
    public record FrcBodySnapshot(
        Pose2d pose,
        Mass mass,
        MomentOfInertia momentOfInertia,
        Velocity2d velocity,
        AngularVelocity angularVelocity,
        double linearDamping,
        double angularDamping,
        double gravityScale,
        boolean isBullet,
        double atRestTime,
        Translation2d forces,
        Torque torque,
        Translation2d accumulatedForce,
        Torque accumulatedTorque
    ) implements StructSerializable {
        public static final Struct<FrcBodySnapshot> struct = ProceduralStructGenerator.genRecord(FrcBodySnapshot.class);
    }

    public FrcBodySnapshot snapshot() {
        return new FrcBodySnapshot(
            GeometryConvertor.toWpilibPose2d(getTransform()),
            GeometryConvertor.toWpilibUnit(getMass()).getFirst(),
            GeometryConvertor.toWpilibUnit(getMass()).getSecond(),
            new Velocity2d(getLinearVelocity().x, getLinearVelocity().y),
            RadiansPerSecond.of(-getAngularVelocity()),
            getLinearDamping(),
            getAngularDamping(),
            getGravityScale(),
            isBullet(),
            this.atRestTime,
            new Translation2d(getForce().x, getForce().y),
            NewtonMeters.of(getTorque()),
            new Translation2d(getAccumulatedForce().x, getAccumulatedForce().y),
            NewtonMeters.of(getAccumulatedTorque())
        );
    }
}
