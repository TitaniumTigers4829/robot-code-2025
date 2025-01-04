// package frc.robot.extras.characterization;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.SwerveConstants.TrajectoryConstants;
// import frc.robot.subsystems.swerve.SwerveDrive;

// public class WheelRadiusCharacterization extends Command {
//   private SwerveDrive swerveSubsystem;
//   private static final double characterizationSpeed = 0.1;
//   private static final double driveRadius = TrajectoryConstants.DRIVE_BASE_DIAMETER;

//   public enum Direction {
//     CLOCKWISE(-1),
//     COUNTER_CLOCKWISE(1);

//     private final int value;

//     Direction(int value) {
//       this.value = value;
//     }
//   }

//   private final double omegaDirection;
//   private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

//   private double lastGyroYawRads = 0.0;
//   private double accumGyroYawRads = 0.0;

//   private double[] startWheelPositions;

//   private double currentEffectiveWheelRadius = 0.0;

//   public WheelRadiusCharacterization(SwerveDrive driveSubsystem, Direction omegaDirection) {
//     this.swerveSubsystem = driveSubsystem;
//     this.omegaDirection = omegaDirection.value;
//     addRequirements(driveSubsystem);
//   }

//   @Override
//   public void initialize() {
//     // Reset
//     swerveSubsystem.zeroHeading();
//     startWheelPositions = swerveSubsystem.getWheelRadiusCharacterizationPosition();
//     lastGyroYawRads = swerveSubsystem.getGyroRotation2d().getRadians();

//     omegaLimiter.reset(0);
//   }

//   @Override
//   public void execute() {
//     // Run drive at velocity
//     swerveSubsystem.runWheelRadiusCharacterization(
//         omegaLimiter.calculate(omegaDirection * characterizationSpeed));

//     // Get yaw and wheel positions
//     accumGyroYawRads = swerveSubsystem.getGyroRotation2d().getRadians() - lastGyroYawRads;
//     // accumGyroYawRads +=
// MathUtil.angleModulus(driveSubsystem.getPose().getRotation().getRadians()
//     // - lastGyroYawRads);
//     double averageWheelPosition = 0.0;
//     double[] wheelPositiions = swerveSubsystem.getWheelRadiusCharacterizationPosition();
//     for (int i = 0; i < 4; i++) {
//       averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
//     }
//     averageWheelPosition /= 4.0;

//     currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
//     SmartDashboard.putNumber("Drive/RadiusCharacterization/driveRadius", driveRadius);
//     SmartDashboard.putNumber("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
//     SmartDashboard.putNumber("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
//     SmartDashboard.putNumber(
//         "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
//         Units.metersToInches(currentEffectiveWheelRadius));
//   }

//   @Override
//   public void end(boolean interrupted) {
//     if (accumGyroYawRads <= Math.PI * 2.0) {
//       System.out.println("Not enough data for characterization");
//     } else {
//       System.out.println(
//           "Effective Wheel Radius: "
//               + Units.metersToInches(currentEffectiveWheelRadius)
//               + " inches");
//       SmartDashboard.putNumber(
//           "Effective Wheel Radius (inches): ",
// Units.metersToInches(currentEffectiveWheelRadius));
//     }
//   }
// }
