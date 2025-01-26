package frc.robot.extras.sim.configs;

import edu.wpi.first.util.struct.StructSerializable;

public class SimSwerveModuleConfig implements StructSerializable {
  public SimMechanismConfig driveConfig;
  public SimMechanismConfig steerConfig;
  public double tireCoefficientOfFriction;
  public double wheelsRadiusMeters;

  public SimSwerveModuleConfig(
      SimMechanismConfig driveConfig,
      SimMechanismConfig steerConfig,
      double tireCoefficientOfFriction,
      double wheelsRadiusMeters) {
    this.driveConfig = driveConfig;
    this.steerConfig = steerConfig;
    this.tireCoefficientOfFriction = tireCoefficientOfFriction;
    this.wheelsRadiusMeters = wheelsRadiusMeters;
  }

  /** Stores the coefficient of friction of some common used wheels. */
  public enum WheelCof {
    // https://www.chiefdelphi.com/t/spectrum-3847-build-blog-2024/447471/217
    COLSONS(0.9),
    BLACK_NITRILE(1.542),
    VEX_GRIPLOCK_V2(1.916);

    public final double cof;

    WheelCof(double cof) {
      this.cof = cof;
    }
  }

  // public static final Struct<SimSwerveModuleConfig> struct =
  // StructGenerator.addCustomStruct(SimSwerveModuleConfig.class);

  // /**
  //  * creates a <a
  // href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS
  // Mark4
  //  * Swerve Module</a> for simulation
  //  */
  // public static SwerveModuleConfig ofMk4(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 8.14;
  //                 case 2 -> 6.75;
  //                 case 3 -> 6.12;
  //                 case 4 -> 5.14;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             12.8,
  //             0.2,
  //             0.3,
  //             wheelCOF,
  //             Units.inchesToMeters(2),
  //             0.03);
  // }

  // /**
  //  * creates a <a
  // href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS
  //  * Mark4-i Swerve Module</a> for simulation
  //  */
  // public static SwerveModuleConfig ofMk4i(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 8.14;
  //                 case 2 -> 6.75;
  //                 case 3 -> 6.12;
  //                 case 4 -> 5.15;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             150.0 / 7.0,
  //             0.2,
  //             1,
  //             wheelCOF,
  //             Units.inchesToMeters(2),
  //             0.025);
  // }

  // /**
  //  * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS
  // Mark4-n Swerve
  //  * Module</a> for simulation
  //  */
  // public static SwerveModuleConfig ofMk4n(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 7.13;
  //                 case 2 -> 5.9;
  //                 case 3 -> 5.36;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             18.75,
  //             0.25,
  //             1,
  //             wheelCOF,
  //             Units.inchesToMeters(2),
  //             0.025);
  // }

  // /**
  //  * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x">WCP
  // SwerveX Swerve Module</a>
  //  * for simulation
  //  *
  //  * <p>X1 Ratios are gearRatioLevel 1-3 <br>
  //  * X2 Ratios are gearRatioLevel 4-6 <br>
  //  * X3 Ratios are gearRatioLevel 7-9
  //  */
  // public static SwerveModuleConfig ofSwerveX(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 7.85;
  //                 case 2 -> 7.13;
  //                 case 3 -> 6.54;
  //                 case 4 -> 6.56;
  //                 case 5 -> 5.96;
  //                 case 6 -> 5.46;
  //                 case 7 -> 5.14;
  //                 case 8 -> 4.75;
  //                 case 9 -> 4.41;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             11.3142,
  //             0.2,
  //             0.3,
  //             wheelCOF,
  //             Units.inchesToMeters(2),
  //             0.03);
  // }

  // /**
  //  * creates a <a
  // href="https://wcproducts.com/collections/gearboxes/products/swerve-x-flipped">WCP SwerveX
  // Flipped
  //  * Swerve Module</a> for simulation
  //  *
  //  * <p>X1 Ratios are gearRatioLevel 1-3 <br>
  //  * X2 Ratios are gearRatioLevel 4-6 <br>
  //  * X3 Ratios are gearRatioLevel 7-9
  //  */
  // public static SwerveModuleConfig ofSwerveXFlipped(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 8.1;
  //                 case 2 -> 7.36;
  //                 case 3 -> 6.75;
  //                 case 4 -> 6.72;
  //                 case 5 -> 6.11;
  //                 case 6 -> 5.6;
  //                 case 7 -> 5.51;
  //                 case 8 -> 5.01;
  //                 case 9 -> 4.59;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             11.3714,
  //             0.2,
  //             0.3,
  //             wheelCOF,
  //             Units.inchesToMeters(2),
  //             0.03);
  // }

  // /**
  //  * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP
  // SwerveXS Swerve
  //  * Module</a> for simulation
  //  *
  //  * <p>X1 Ratios are gearRatioLevel 1-3 <br>
  //  * X2 Ratios are gearRatioLevel 4-6
  //  */
  // public static SwerveModuleConfig ofSwerveXS(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 6.0;
  //                 case 2 -> 5.54;
  //                 case 3 -> 5.14;
  //                 case 4 -> 4.71;
  //                 case 5 -> 4.4;
  //                 case 6 -> 4.13;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             41.25,
  //             0.2,
  //             0.3,
  //             wheelCOF,
  //             Units.inchesToMeters(1.5),
  //             0.03);
  // }

  // /**
  //  * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2">WCP
  // SwerveX2 Swerve
  //  * Module</a> for simulation
  //  *
  //  * <p>X1 Ratios are gearRatioLevel 1-3 <br>
  //  * X2 Ratios are gearRatioLevel 4-6 <br>
  //  * X3 Ratios are gearRatioLevel 7-9 <br>
  //  * X4 Ratios are gearRatioLevel 10-12
  //  */
  // public static SwerveModuleConfig ofSwerveX2(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 7.67;
  //                 case 2 -> 6.98;
  //                 case 3 -> 6.39;
  //                 case 4 -> 6.82;
  //                 case 5 -> 6.20;
  //                 case 6 -> 5.68;
  //                 case 7 -> 6.48;
  //                 case 8 -> 5.89;
  //                 case 9 -> 5.40;
  //                 case 10 -> 5.67;
  //                 case 11 -> 5.15;
  //                 case 12 -> 4.73;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             12.1,
  //             0.2,
  //             0.3,
  //             wheelCOF,
  //             Units.inchesToMeters(2),
  //             0.03);
  // }

  // /**
  //  * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP
  // SwerveX2S Swerve
  //  * Module</a> for simulation
  //  *
  //  * <p>X1 Ratios are gearRatioLevel 1-3 <br>
  //  * X2 Ratios are gearRatioLevel 4-6 <br>
  //  * X3 Ratios are gearRatioLevel 7-9
  //  */
  // public static SwerveModuleConfig ofSwerveX2S(
  //         DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF,
  // int gearRatioLevel) {
  //     return new SwerveModuleConfig(
  //             driveMotor,
  //             steerMotor,
  //             driveCurrentLimitAmps,
  //             switch (gearRatioLevel) {
  //                 case 1 -> 6.0;
  //                 case 2 -> 5.63;
  //                 case 3 -> 5.29;
  //                 case 4 -> 4.94;
  //                 case 5 -> 4.67;
  //                 case 6 -> 4.42;
  //                 case 7 -> 4.11;
  //                 case 8 -> 3.9;
  //                 case 9 -> 3.71;
  //                 default -> throw new IllegalStateException("Unknown gearing level: " +
  // gearRatioLevel);
  //             },
  //             25.9,
  //             0.2,
  //             0.3,
  //             wheelCOF,
  //             Units.inchesToMeters(1.875),
  //             0.03);
  // }
}
