package frc.robot.extras.swerve.setpointGen;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.util.ProceduralStructGenerator;
import frc.robot.extras.util.ProceduralStructGenerator.FixedSizeArray;
import frc.robot.extras.util.ProceduralStructGenerator.IgnoreStructField;
import java.nio.ByteBuffer;
import java.util.Arrays;

class SPGCalcs {
  private static final double kEpsilon = 1E-8;
  static final int NUM_MODULES = 4;

  static double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  static double findSteeringMaxS(
      double prevVx,
      double prevVy,
      double prevHeading,
      double desiredVx,
      double desiredVy,
      double desiredHeading,
      double maxDeviation) {
    desiredHeading = unwrapAngle(prevHeading, desiredHeading);
    double diff = desiredHeading - prevHeading;
    if (Math.abs(diff) <= maxDeviation) {
      // Can go all the way to s=1.
      return 1.0;
    }

    double target = prevHeading + Math.copySign(maxDeviation, diff);

    // Rotate the velocity vectors such that the target angle becomes the +X
    // axis. We only need find the Y components, h_0 and h_1, since they are
    // proportional to the distances from the two points to the solution
    // point (x_0 + (x_1 - x_0)s, y_0 + (y_1 - y_0)s).
    double sin = Math.sin(-target);
    double cos = Math.cos(-target);
    double h_0 = sin * prevVx + cos * prevVy;
    double h_1 = sin * desiredVx + cos * desiredVy;

    // Undo linear interpolation from h_0 to h_1:
    // 0 = h_0 + (h_1 - h_0) * s
    // -h_0 = (h_1 - h_0) * s
    // -h_0 / (h_1 - h_0) = s
    // h_0 / (h_0 - h_1) = s
    // Guaranteed to not divide by zero, since if h_0 was equal to h_1, theta_0
    // would be equal to theta_1, which is caught by the difference check.
    return h_0 / (h_0 - h_1);
  }

  private static boolean isValidS(double s) {
    return Double.isFinite(s) && s >= 0 && s <= 1;
  }

  static double findDriveMaxS(double x_0, double y_0, double x_1, double y_1, double max_vel_step) {
    // Derivation:
    // Want to find point P(s) between (x_0, y_0) and (x_1, y_1) where the
    // length of P(s) is the target T. P(s) is linearly interpolated between the
    // points, so P(s) = (x_0 + (x_1 - x_0) * s, y_0 + (y_1 - y_0) * s).
    // Then,
    //     T = sqrt(P(s).x^2 + P(s).y^2)
    //   T^2 = (x_0 + (x_1 - x_0) * s)^2 + (y_0 + (y_1 - y_0) * s)^2
    //   T^2 = x_0^2 + 2x_0(x_1-x_0)s + (x_1-x_0)^2*s^2
    //       + y_0^2 + 2y_0(y_1-y_0)s + (y_1-y_0)^2*s^2
    //   T^2 = x_0^2 + 2x_0x_1s - 2x_0^2*s + x_1^2*s^2 - 2x_0x_1s^2 + x_0^2*s^2
    //       + y_0^2 + 2y_0y_1s - 2y_0^2*s + y_1^2*s^2 - 2y_0y_1s^2 + y_0^2*s^2
    //     0 = (x_0^2 + y_0^2 + x_1^2 + y_1^2 - 2x_0x_1 - 2y_0y_1)s^2
    //       + (2x_0x_1 + 2y_0y_1 - 2x_0^2 - 2y_0^2)s
    //       + (x_0^2 + y_0^2 - T^2).
    //
    // To simplify, we can factor out some common parts:
    // Let l_0 = x_0^2 + y_0^2, l_1 = x_1^2 + y_1^2, and
    // p = x_0 * x_1 + y_0 * y_1.
    // Then we have
    //   0 = (l_0 + l_1 - 2p)s^2 + 2(p - l_0)s + (l_0 - T^2),
    // with which we can solve for s using the quadratic formula.

    double l_0 = x_0 * x_0 + y_0 * y_0;
    double l_1 = x_1 * x_1 + y_1 * y_1;
    double sqrt_l_0 = Math.sqrt(l_0);
    double diff = Math.sqrt(l_1) - sqrt_l_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }

    double target = sqrt_l_0 + Math.copySign(max_vel_step, diff);
    double p = x_0 * x_1 + y_0 * y_1;

    // Quadratic of s
    double a = l_0 + l_1 - 2 * p;
    double b = 2 * (p - l_0);
    double c = l_0 - target * target;
    double root = Math.sqrt(b * b - 4 * a * c);

    // Check if either of the solutions are valid
    // Won't divide by zero because it is only possible for a to be zero if the
    // target velocity is exactly the same or the reverse of the current
    // velocity, which would be caught by the difference check.
    double s_1 = (-b + root) / (2 * a);
    if (isValidS(s_1)) {
      return s_1;
    }
    double s_2 = (-b - root) / (2 * a);
    if (isValidS(s_2)) {
      return s_2;
    }

    // Since we passed the initial max_vel_step check, a solution should exist,
    // but if no solution was found anyway, just don't limit movement
    return 1.0;
  }

  static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
    return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
        && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
        && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  static boolean flipHeading(double prevToGoal) {
    return Math.abs(prevToGoal) > Math.PI / 2.0;
  }

  private static final Struct<LocalVectors> structVectors;
  private static final Struct<LocalVars> structVars;

  static {
    final var structVectorsProc =
        ProceduralStructGenerator.genObject(LocalVectors.class, LocalVectors::new);
    structVectors =
        new Struct<SPGCalcs.LocalVectors>() {
          @Override
          public String getSchema() {
            return "float64 vx;float64 vy;float64 cos;float64 sin;";
          }

          @Override
          public int getSize() {
            return 32;
          }

          @Override
          public Class<LocalVectors> getTypeClass() {
            return LocalVectors.class;
          }

          @Override
          public String getTypeName() {
            return "LocalVectors";
          }

          @Override
          public void pack(ByteBuffer bb, LocalVectors value) {
            bb.putDouble(value.vx);
            bb.putDouble(value.vy);
            bb.putDouble(value.cos);
            bb.putDouble(value.sin);
          }

          @Override
          public LocalVectors unpack(ByteBuffer bb) {
            return structVectorsProc.unpack(bb);
          }

          @Override
          public String toString() {
            return this.getTypeName() + "<" + this.getSize() + ">" + " {" + this.getSchema() + "}";
          }
        };
    structVars = ProceduralStructGenerator.genObject(LocalVars.class, LocalVars::new);
    System.out.println(structVectors);
    System.out.println(structVars);
  }

  static final class LocalVectors implements StructSerializable {
    public double vx, vy, cos, sin;

    public LocalVectors() {}

    public void reset() {
      vx = vy = cos = sin = 0.0;
    }

    public void applyModuleState(SwerveModuleState state) {
      cos = state.angle.getCos();
      sin = state.angle.getSin();
      vx = cos * state.speedMetersPerSecond;
      vy = sin * state.speedMetersPerSecond;
      if (state.speedMetersPerSecond < 0.0) {
        applyRotation(Rotation2d.k180deg.getCos(), Rotation2d.k180deg.getSin());
      }
    }

    public LocalVectors applyRotation(double otherCos, double otherSin) {
      double newCos = cos * otherCos - sin * otherSin;
      double newSin = cos * otherSin + sin * otherCos;
      cos = newCos;
      sin = newSin;

      return this;
    }

    public double radians() {
      return Math.atan2(sin, cos);
    }

    public static final Struct<LocalVectors> struct = structVectors;
  }

  static final class LocalVars implements StructSerializable {
    @FixedSizeArray(size = NUM_MODULES)
    public LocalVectors[] prev;

    @FixedSizeArray(size = NUM_MODULES)
    public LocalVectors[] desired;

    public boolean needToSteer = true, allModulesShouldFlip = true;
    public double minS, dt;
    public double dx, dy, dtheta;
    public ChassisSpeeds prevSpeeds, desiredSpeeds;

    @FixedSizeArray(size = NUM_MODULES)
    public SwerveModuleState[] prevModuleStates;

    @FixedSizeArray(size = NUM_MODULES)
    public SwerveModuleState[] desiredModuleStates;

    @IgnoreStructField public Rotation2d[] steeringOverride;

    public LocalVars() {
      desiredSpeeds = prevSpeeds = new ChassisSpeeds();
      prev = new LocalVectors[NUM_MODULES];
      desired = new LocalVectors[NUM_MODULES];
      steeringOverride = new Rotation2d[NUM_MODULES];
      for (int i = 0; i < NUM_MODULES; i++) {
        prev[i] = new LocalVectors();
        desired[i] = new LocalVectors();
      }
    }

    public LocalVars reset() {
      needToSteer = allModulesShouldFlip = true;
      Arrays.fill(steeringOverride, null);
      for (int i = 0; i < NUM_MODULES; i++) {
        prev[i].reset();
        desired[i].reset();
      }

      return this;
    }

    public static final Struct<LocalVars> struct = structVars;
  }
}
