package frc.robot.extras.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static sham.utils.mathutils.MeasureMath.div;

import java.util.Random;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import sham.ShamArena.ShamEnvTiming;
import sham.utils.GearRatio;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.MeasureMath;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import monologue.ProceduralStructGenerator;

public class SimMechanism {
    private static final double kMotorEfficiency = 0.85;

    private static final Random RAND = new Random();

    private static final AngleUnit Rad = Radians;
    private static final AngularVelocityUnit RadPS = RadiansPerSecond;
    private static final AngularAccelerationUnit RadPS2 = RadiansPerSecondPerSecond;

    /**
     * An interface to define systemic impacts on a mechanism.
     * 
     * <p>This could be the force of gravity, a spring, inertial load due to chassis
     * acceleration, etc.
     * 
     * <p>It is sound behavior for this to capture outputs of other mechanisms but
     * be aware the order of calculations between mechanisms is not guaranteed.
     * For example, if an inverse pendulum mechanism could have extra load depending
     * on the drive mechanism's acceleration, the drive mechanism should be calculated
     * first in the same time slot but this can not be promised.
     */
    public interface MechanismDynamics {
        default Torque environment(Angle angle, AngularVelocity velocity) {
            return NewtonMeters.zero();
        }

        default MomentOfInertia extraInertia() {
            return KilogramSquareMeters.zero();
        }

        static MechanismDynamics of(Torque environment) {
            return new MechanismDynamics() {
                @Override
                public Torque environment(Angle angle, AngularVelocity velocity) {
                    return environment;
                }

                @Override
                public MomentOfInertia extraInertia() {
                    return KilogramSquareMeters.zero();
                }
            };
        }

        static MechanismDynamics zero() {
            return MechanismDynamics.of(NewtonMeters.zero());
        }
    }

    /**
     * A record to define the friction of a mechanism.
     * The only way to obtain this value is to empirically measure it.
     * This record allows defining separate static and kinetic friction values
     * but it is perfectly valid to use the same value for both.
     * 
     * <p> Creating a friction using {@code kS} is a valid approach and can
     * be done like so:
     * <pre><code>
     * //could be any motor
     *DCMotor motor = DCMotor.getFalcon500(1);
     *Friction.of(motor, Volts.of(kMechanism.kS));
     * </code></pre>
     */
    public record Friction(Torque staticFriction, Torque kineticFriction) implements StructSerializable {
        public static Friction of(Torque staticFriction, Torque kineticFriction) {
            return new Friction(staticFriction, kineticFriction);
        }

        public static Friction of(Torque universalFriction) {
            return new Friction(universalFriction, universalFriction);
        }

        public static Friction of(DCMotor motor, Voltage staticVoltage, Voltage kineticVoltage) {
            Torque staticTorque = NewtonMeters.of(
                motor.getTorque(motor.getCurrent(0.0, staticVoltage.in(Volts))));
            Torque kineticTorque = NewtonMeters.of(
                motor.getTorque(motor.getCurrent(0.0, kineticVoltage.in(Volts))));
            return new Friction(staticTorque, kineticTorque);
        }

        public static Friction of(DCMotor motor, Voltage universalVoltage) {
            return of(motor, universalVoltage, universalVoltage);
        }

        public static Friction zero() {
            return new Friction(NewtonMeters.zero(), NewtonMeters.zero());
        }

        public static final Struct<Friction> struct = ProceduralStructGenerator.genRecord(Friction.class);
    }

    public record HardLimits(Angle minAngle, Angle maxAngle) implements StructSerializable {
        public static HardLimits of(Angle minAngle, Angle maxAngle) {
            return new HardLimits(minAngle, maxAngle);
        }

        public static HardLimits unbounded() {
            return new HardLimits(Rad.of(-1_000_000_000_000.0), Rad.of(1_000_000_000_000.0));
        }

        public static final Struct<HardLimits> struct = ProceduralStructGenerator.genRecord(HardLimits.class);
    }

    /**
     * A record to define the output of a mechanism.
     * 
     * <p> All measures in this record are immutable.
     */
    public record MechanismOutputs(
        Angle position,
        AngularVelocity velocity,
        AngularAcceleration acceleration) implements StructSerializable {

        public static MechanismOutputs of(Angle angle, AngularVelocity velocity, AngularAcceleration acceleration) {
            return new MechanismOutputs(angle, velocity, acceleration);
        }

        public static MechanismOutputs zero() {
            return new MechanismOutputs(Rad.zero(), RadPS.zero(), RadPS2.zero());
        }

        public MechanismOutputs times(double scalar) {
            return new MechanismOutputs(position.times(scalar), velocity.times(scalar), acceleration.times(scalar));
        }

        public static final Struct<MechanismOutputs> struct = ProceduralStructGenerator.genRecord(MechanismOutputs.class);
    }

    /**
     * A record to define the inputs of a mechanism.
     * 
     * <p> All measures in this record are immutable.
     */
    public record MechanismInputs(
        Torque torque,
        Voltage statorVoltage,
        Voltage supplyVoltage,
        Current statorCurrent) implements StructSerializable {

        public Current supplyCurrent() {
            // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10;
            return statorCurrent.times(statorVoltage.div(supplyVoltage)).div(kMotorEfficiency);
        }

        public static MechanismInputs of(Torque torque, Voltage voltage, Voltage supplyVoltage, Current statorCurrent) {
            return new MechanismInputs(torque, voltage, supplyVoltage, statorCurrent);
        }

        public static MechanismInputs zero() {
            return new MechanismInputs(NewtonMeters.zero(), Volts.zero(), Volts.zero(), Amps.zero());
        }

        public static final Struct<MechanismInputs> struct = ProceduralStructGenerator.genRecord(MechanismInputs.class);
    }

    private final String name;
    private final DataLogger logger;
    private final MechanismDynamics dynamics;
    private final Friction friction;
    private final GearRatio gearRatio;
    private final DCMotor motor;
    private final SimMotorController controller;
    private final ShamEnvTiming timing;
    private final MomentOfInertia rotorInertia;
    private final HardLimits limits;
    private final double noise;

    private final ReentrantReadWriteLock ioLock = new ReentrantReadWriteLock();
    private MechanismOutputs outputs = MechanismOutputs.zero();
    private MechanismInputs inputs = MechanismInputs.zero();

    public SimMechanism(
        String name,
        DCMotor motor,
        SimMotorController controller,
        MomentOfInertia rotorInertia,
        GearRatio gearRatio,
        Friction friction,
        MechanismDynamics dynamics,
        HardLimits limits,
        double noise,
        ShamEnvTiming timing
    ) {
        this.logger = RuntimeLog.loggerFor("Mechanism/"+name);
        this.dynamics = dynamics;
        this.friction = friction;
        this.gearRatio = gearRatio;
        this.motor = motor;
        this.controller = controller;
        this.timing = timing;
        this.rotorInertia = rotorInertia;
        this.name = name;
        this.limits = limits;
        this.noise = noise;

        logger.log("dynamics", dynamics.getClass().getSimpleName());
        logger.log("friction", friction, Friction.struct);
        logger.log("gearRatio", gearRatio, GearRatio.struct);
        logger.log("motor", motor, DCMotor.struct);
        logger.log("controller", controller.getClass().getSimpleName());
        logger.log("rotorInertia", rotorInertia);
        logger.log("limits", limits, HardLimits.struct);
        logger.log("noise", noise);
    }

    protected Torque getBrakingTorque() {
        if (!controller.brakeEnabled()) {
            logger.log("Update/braking", false);
            logger.log("Update/brakingTorque", 0.0);
            logger.log("Update/brakingCurrent", 0.0);
            return NewtonMeters.zero();
        }
        double current = motor.getCurrent(outputs.velocity().in(RadPS), Volts.zero().in(Volts));
        double torque = motor.getTorque(current);
        logger.log("Update/braking", true);
        logger.log("Update/brakingTorque", torque);
        logger.log("Update/brakingCurrent", current);
        return NewtonMeters.of(torque);
    }

    /**
     * Applies friction and braking to the output of the mechanism.
     * This will reduce the torque applied to the mechanism.
     * 
     * @param torque the torque to apply to the mechanism
     * @return the torque after friction has been applied
     */
    protected Torque antiTorque(Torque motor, Torque environment, MomentOfInertia inertia) {
        AngularVelocity velocity = outputs().velocity();
        boolean isMoving = MeasureMath.abs(velocity).gt(RadPS.zero());
        logger.log("Update/moving", isMoving);
        Torque frictionTorque = isMoving ? friction.kineticFriction() : friction.staticFriction();
        Torque brakingTorque = NewtonMeters.zero();
        if (MeasureMath.abs(motor).lt(NewtonMeters.of(0.01))) {
            brakingTorque = getBrakingTorque();
        }
        // ensure that friction/braking opposes the motion
        Torque antiTorque = frictionTorque.plus(brakingTorque)
                .times(-MeasureMath.signum(velocity) * gearRatio.getReduction());

        logger.log("Update/frictionTorque", frictionTorque);
        logger.log("Update/desiredAntiTorque", antiTorque);

        final AngularAcceleration unburdenedAccel = div(motor.plus(environment), inertia);
        final AngularAcceleration burdenAccel = div(antiTorque, inertia);
        final AngularAcceleration imposedAccel = unburdenedAccel.plus(burdenAccel);
        logger.log("Update/unburdenedAccel", unburdenedAccel);
        logger.log("Update/burdenAccel", burdenAccel);
        logger.log("Update/imposedAccel", imposedAccel);

        // the goal of anti torque is to reduce the velocity to 0
        // so we need to ensure that the anti torque is not so large that it causes the mechanism to reverse

        final AngularAcceleration accelNeededToStop = velocity.times(-1.0).div(timing.dt());
        final Torque torqueNeededToStop = MeasureMath.times(accelNeededToStop, inertia);
        logger.log("Update/accelNeededToStop", accelNeededToStop);

        if (accelNeededToStop.lt(RadPS2.zero())) {
            // accel needed to stop is negative
            if (imposedAccel.lt(accelNeededToStop)) {
                // the imposed accel has a greater magnitude in the same sign
                // as the accel needed to stop, this will cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", torqueNeededToStop);
                return torqueNeededToStop;
            } else {
                // the imposed accel has a lesser magnitude in the same sign
                // as the accel needed to stop, this will cannot cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", antiTorque);
                return antiTorque;
            }
        } else {
            // accel needed to stop is positive
            if (imposedAccel.gt(accelNeededToStop)) {
                // the imposed accel has a greater magnitude in the same sign
                // as the accel needed to stop, this will cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", torqueNeededToStop);
                return torqueNeededToStop;
            } else {
                // the imposed accel has a lesser magnitude in the same sign
                // as the accel needed to stop, this will cannot cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", antiTorque);
                return antiTorque;
            }
        }
    }

    /**
     * Gets the motors output voltage based on the current state of the mechanism.
     * 
     * @return the voltage to apply to the motor
     */
    protected Voltage getControllerVoltage(Voltage supplyVoltage) {
        Voltage rv = controller.run(timing.dt(), supplyVoltage, outputs());
        Voltage v = MeasureMath.clamp(rv, RobotController.getMeasureBatteryVoltage().times(-1.0), RobotController.getMeasureBatteryVoltage());
        if (DriverStation.isDisabled()) {
            return Volts.zero();
        }
        return v;
    }

    protected Torque getMotorTorque(Voltage voltage) {
        if (voltage.isNear(Volts.zero(), Volts.of(0.01))) {
            return NewtonMeters.zero();
        }
        double current = motor.getCurrent(motorOutputs().velocity().in(RadPS), voltage.in(Volts));
        logger.log("Update/current", current);
        double torque = motor.getTorque(MathUtil.clamp(current, -80.0, 80.0));
        return NewtonMeters.of(torque).times(gearRatio.getReduction());
    }

    /**
     * Gets the current state of the mechanism.
     * 
     * @return the current state of the mechanism
     */
    public MechanismOutputs outputs() {
        try {
            ioLock.readLock().lock();
            return outputs;
        } finally {
            ioLock.readLock().unlock();
        }
    }

    /**
     * Gets the current state of the motor driving the mechanism.
     * 
     * @return the current state of the mechanism
     */
    public MechanismOutputs motorOutputs() {
        try {
            ioLock.readLock().lock();
            return outputs.times(gearRatio.getReduction());
        } finally {
            ioLock.readLock().unlock();
        }
    }

    public MechanismInputs inputs() {
        try {
            ioLock.readLock().lock();
            return inputs;
        } finally {
            ioLock.readLock().unlock();
        }
    }

    public void overrideOutputs(MechanismOutputs outputs) {
        try {
            ioLock.writeLock().lock();
            this.outputs = outputs;
        } finally {
            ioLock.writeLock().unlock();
        }
    }

    /**
     * Gets the name of the mechanism.
     * 
     * @return the name of the mechanism
     */
    public String name() {
        return name;
    }

    /**
     * Updates the state of the mechanism.
     */
    void update(final Voltage supplyVoltage) {
        final Time dt = timing.dt();

        final MomentOfInertia inertia = rotorInertia.plus(dynamics.extraInertia());
        logger.log("Update/inertia", inertia);

        // run the motor controller loop
        final Voltage voltage = getControllerVoltage(supplyVoltage);
        logger.log("Update/voltage", voltage);

        // calculate the torque acting on the mechanism
        final Torque environment = dynamics.environment(outputs().position(), outputs().velocity());
        final Torque motorTorque = getMotorTorque(voltage);
        final Torque antiTorque = antiTorque(motorTorque, environment, inertia);
        final Torque outputTorque = motorTorque.plus(antiTorque);
        logger.log("Update/environmentTorque", environment);
        logger.log("Update/motorTorque", motorTorque);
        logger.log("Update/antiTorque", antiTorque);
        logger.log("Update/outputTorque", outputTorque);

        // calculate the displacement, velocity, and acceleration of the mechanism
        final AngularAcceleration acceleration = div(outputTorque, inertia)
            .times(1.0 + (noise * RAND.nextGaussian()));
        final AngularVelocity velocity = MeasureMath.nudgeZero(
            outputs().velocity().plus(acceleration.times(dt)),
            RotationsPerSecond.of(0.001)
        );
        final Angle angle = outputs().position().plus(velocity.times(dt));

        try {
            ioLock.writeLock().lock();
            // apply hard limits
            // then update the state accordingly
            if (angle.lt(limits.minAngle())) {
                outputs = MechanismOutputs.of(limits.minAngle(), RadPS.zero(), RadPS2.zero());
            } else if (angle.gt(limits.maxAngle())) {
                outputs = MechanismOutputs.of(limits.maxAngle(), RadPS.zero(), RadPS2.zero());
            } else {
                outputs = MechanismOutputs.of(angle, velocity, acceleration);
            }
            // capture the "inputs"
            inputs = MechanismInputs.of(
                motorTorque, voltage, supplyVoltage,
                Amps.of(motor.getCurrent(outputs.velocity.in(RadPS), voltage.in(Volts)))
            );
        } finally {
            logger.log("Update/outputs", outputs, MechanismOutputs.struct);
            logger.log("Update/motorOutputs", motorOutputs(), MechanismOutputs.struct);
            logger.log("Update/inputs", inputs, MechanismInputs.struct);
            ioLock.writeLock().unlock();
        }
    }
}
