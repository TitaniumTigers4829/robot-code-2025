
package frc.robot.extras.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.extras.sim.SimArena.SimEnvTiming;
import frc.robot.extras.sim.configs.SimGyroConfig;
import frc.robot.extras.sim.utils.RuntimeLog;
import frc.robot.extras.sim.utils.mathutils.MeasureMath.XY;
import frc.robot.extras.sim.utils.mathutils.SimCommonMath;

import java.util.function.BiConsumer;

// import sham.utils.mathutils.MeasureMath;

public class SimGyro {
  // /* The threshold of instantaneous angular acceleration at which the chassis is considered to
  // experience an "impact." */
  // private static final AngularAcceleration START_DRIFTING = RadiansPerSecondPerSecond.of(500);
  // /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of
  // the angular acceleration threshold. */
  // private static final Angle DRIFT_DUE_TO_IMPACT_COEFFICIENT = Radians.of(1);

  private final SimEnvTiming timing;
  private BiConsumer<Pair<Angle, AngularVelocity>, XY<LinearAcceleration>> updateConsumer;

  private final double veloStdDev;

  private final AngularVelocity averageDriftingMotionless;

  private Twist2d lastTwist = new Twist2d();

  public SimGyro(SimEnvTiming timing, SimGyroConfig gyroConfig) {
    this.timing = timing;
    this.averageDriftingMotionless =
        Degrees.of(gyroConfig.averageDriftingIn30SecsMotionlessDeg).div(Seconds.of(30.0));
    this.veloStdDev = gyroConfig.velocityMeasurementStandardDeviationPercent;

    RuntimeLog.debug("Created a swerve module simulation");
  }

  public void setUpdateConsumer(
      BiConsumer<Pair<Angle, AngularVelocity>, XY<LinearAcceleration>> updateConsumer) {
    this.updateConsumer = updateConsumer;
  }

  /**
   *
   *
   * <h2>Updates the Gyro Simulation for Each Sub-Tick.</h2>
   *
   * <p>This method updates the gyro simulation and should be called during every sub-tick of the
   * simulation.
   *
   * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second,
   *     usually obtained from {@link ShamDriveTrain#getAngularVelocity()}
   */
  public void updateSimulationSubTick(Angle angleThisTick, Twist2d twistThisTick) {
    AngularVelocity actualAngularVelocity = Radians.of(twistThisTick.dtheta).div(timing.dt());

    AngularVelocity omegaV =
        actualAngularVelocity
            .plus(averageDriftingMotionless)
            // .plus(getDriftingDueToImpact(actualAngularVelocity))
            .plus(
                actualAngularVelocity.times(SimCommonMath.generateRandomNormal(0.0, veloStdDev)));

    LinearVelocity lastXV = Meters.of(lastTwist.dx).div(timing.dt());
    LinearVelocity lastYV = Meters.of(lastTwist.dy).div(timing.dt());
    LinearVelocity xV = Meters.of(twistThisTick.dx).div(timing.dt());
    LinearVelocity yV = Meters.of(twistThisTick.dy).div(timing.dt());

    LinearAcceleration xA = xV.minus(lastXV).div(timing.dt());
    LinearAcceleration yA = yV.minus(lastYV).div(timing.dt());

    if (updateConsumer != null) {
      updateConsumer.accept(Pair.of(angleThisTick, omegaV), new XY<>(xA, yA));
    }
  }

  // private AngularVelocity getDriftingDueToImpact(AngularVelocity actualAngularVelocity) {
  //     AngularVelocity lastAngularVelocity = RadiansPerSecond.of(
  //         lastTwist.dtheta * timing.dt().in(Seconds)
  //     );
  //     AngularAcceleration angularAcceleration =
  // actualAngularVelocity.minus(lastAngularVelocity).div(timing.dt());
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
