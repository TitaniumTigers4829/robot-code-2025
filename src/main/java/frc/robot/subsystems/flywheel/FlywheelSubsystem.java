// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */
  public TalonFX flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR);
  private static final int TICKS_PER_REV = 2048; // Falcon 500 encoder resolution
  private static final double GEAR_RATIO = 1.0; // Update based on your flywheel gearbox
  private static final double SECONDS_PER_MINUTE = 60.0;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;

  public FlywheelSubsystem(TalonFX flywheelMotor) {
    this.flywheelMotor = flywheelMotor;
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = kP;
    config.slot0.kI = kI;
    config.slot0.kD = kD;
    config.slot0.kF = kF;
    flywheelMotor.configAllSettings(config);
    flywheelMotor.setNeutralMode(NeutralMode.Coast);
    flywheelMotor.setSelectedSensorPosition(0);
  }

  public void setFlywheelSpeed(double rpm) {
        // Convert RPM to encoder ticks per 100ms
        double ticksPer100ms = rpmToTicksPer100ms(rpm);
        flywheelMotor.set(ControlMode.Velocity, ticksPer100ms);
    }

    /**
     * Stops the flywheel.
     */
    public void stop() {
        flywheelMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Gets the current velocity of the flywheel in RPM.
     * @return The flywheel velocity in RPM.
     */
    public double getFlywheelSpeed() {
        double ticksPer100ms = flywheelMotor.getSelectedSensorVelocity();
        return ticksPer100msToRPM(ticksPer100ms);
    }

    private double rpmToTicksPer100ms(double rpm) {
        return (rpm * TICKS_PER_REV * GEAR_RATIO) / SECONDS_PER_MINUTE / 10.0;
    }

    private double ticksPer100msToRPM(double ticksPer100ms) {
        return (ticksPer100ms * SECONDS_PER_MINUTE * 10.0) / (TICKS_PER_REV * GEAR_RATIO);
    }

    @Override
    public void periodic() {
        // Log the current flywheel speed for debugging
        System.out.println("Flywheel Speed (RPM): " + getFlywheelSpeed());
    }
  }
