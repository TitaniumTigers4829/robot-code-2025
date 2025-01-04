package frc.robot.extras.sim;

import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BiConsumer;

import sham.ShamArena.ShamEnvTiming;
import sham.configs.ShamGyroConfig;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.ShamCommonMath;
import sham.utils.mathutils.MeasureMath.XY;
// import sham.utils.mathutils.MeasureMath;


public class SimGyro {
    // /* The threshold of instantaneous angular acceleration at which the chassis is considered to experience an "impact." */
    // private static final AngularAcceleration START_DRIFTING = RadiansPerSecondPerSecond.of(500);
    // /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of the angular acceleration threshold. */
    // private static final Angle DRIFT_DUE_TO_IMPACT_COEFFICIENT = Radians.of(1);

    private final DataLogger logger;

    private final ShamEnvTiming timing;
    private BiConsumer<AngularVelocity, XY<LinearAcceleration>> updateConsumer;

    private final double veloStdDev;

    private final AngularVelocity averageDriftingMotionless;

    private Twist2d lastTwist = new Twist2d();

    public SimGyro(ShamEnvTiming timing, SimGyroConfig gyroConfig, DataLogger logger) {
        this.logger = logger.getSubLogger("Gyro");
        this.timing = timing;
        this.averageDriftingMotionless = Degrees.of(gyroConfig.averageDriftingIn30SecsMotionlessDeg)
                .div(Seconds.of(30.0));
        this.veloStdDev = gyroConfig.velocityMeasurementStandardDeviationPercent;

        this.logger.log("config", gyroConfig, SimGyroConfig.struct);

        RuntimeLog.debug("Created a swerve module simulation");
    }

    public void setUpdateConsumer(BiConsumer<AngularVelocity, XY<LinearAcceleration>> updateConsumer) {
        this.updateConsumer = updateConsumer;
    }

    public Time getDt() {
        return timing.dt();
    }

    /**
     *
     *
     * <h2>Updates the Gyro Simulation for Each Sub-Tick.</h2>
     *
     * <p>This method updates the gyro simulation and should be called during every sub-tick of the simulation.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, usually obtained from
     *     {@link SimDriveTrain#getAngularVelocity()}
     */
    public void updateSimulationSubTick(Twist2d twistThisTick) {
        AngularVelocity actualAngularVelocity = Radians.of(twistThisTick.dtheta)
            .div(timing.dt());

        AngularVelocity omegaV = actualAngularVelocity
                .plus(averageDriftingMotionless)
                // .plus(getDriftingDueToImpact(actualAngularVelocity))
                .plus(actualAngularVelocity
                        .times(ShamCommonMath.generateRandomNormal(0.0, veloStdDev))
                );

        LinearVelocity lastXV = Meters.of(lastTwist.dx).div(timing.dt());
        LinearVelocity lastYV = Meters.of(lastTwist.dy).div(timing.dt());
        LinearVelocity xV = Meters.of(twistThisTick.dx).div(timing.dt());
        LinearVelocity yV = Meters.of(twistThisTick.dy).div(timing.dt());

        LinearAcceleration xA = xV.minus(lastXV).div(timing.dt());
        LinearAcceleration yA = yV.minus(lastYV).div(timing.dt());

        logger.log("omegaV", omegaV);
        logger.log("xA", xA);
        logger.log("yA", yA);

        if (updateConsumer != null) {
            updateConsumer.accept(omegaV, new XY<>(xA, yA));
        }
    }

    // private AngularVelocity getDriftingDueToImpact(AngularVelocity actualAngularVelocity) {
    //     AngularVelocity lastAngularVelocity = RadiansPerSecond.of(
    //         lastTwist.dtheta * timing.dt().in(Seconds)
    //     );
    //     AngularAcceleration angularAcceleration = actualAngularVelocity.minus(lastAngularVelocity).div(timing.dt());
    //     if (MeasureMath.abs(angularAcceleration).gt(START_DRIFTING)) {
    //         return DRIFT_DUE_TO_IMPACT_COEFFICIENT
    //                 .times(MeasureMath.signum(angularAcceleration))
    //                 .times(angularAcceleration.div(START_DRIFTING))
    //                 .div(timing.dt());
    //     } else {
    //         return RadiansPerSecond.of(0);
    //     }
    // }
}
