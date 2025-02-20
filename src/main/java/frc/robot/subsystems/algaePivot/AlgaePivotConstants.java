package frc.robot.subsystems.algaePivot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class AlgaePivotConstants {
  public static final int ALGAE_PIVOT_MOTOR_ID = 0;
  public static final int ALGAE_ENCODER_MOTOR_ID = 0;

  public static final SensorDirectionValue ALGAE_ENCODER_REVERSED =
      SensorDirectionValue.Clockwise_Positive;
  public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 1.0;
  public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND = 1.0;

  public static final double MAX_ANGLE = 5.0;
  public static final double MIN_ANGLE = 0.0;

  public static final double ANGLE_ZERO = 0.0;
  public static final double PIVOT_P = 100.0;
  public static final double PIVOT_I = 0.0;
  public static final double PIVOT_D = 0.0;
  public static final double PIVOT_G = 0.0;

  public static final double ALGAE_GEAR_RATIO = 1.0;
  public static final double ALGAE_MOMENT_INERTIA = 0.01;
  public static final double ALGAE_PIVOT_LENGTH = 1.0;

  public static final double ALGAE_PIVOT_ANGLE = 1.0;
  public static final double ALGAE_PIVOT_SPEED = 1.0;
  public static final double ALGAE_VOLTAGE = 1.0;
  public static final double ALGAE_NEUTRAL_SPEED = 0.0;
}
