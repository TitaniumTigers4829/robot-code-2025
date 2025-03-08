// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int ELEVATOR_LEADER_MOTOR_ID = 0;
  public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 62;

  public static final double ELEVATOR_P = 110;
  public static final double ELEVATOR_I = 0;
  public static final double ELEVATOR_D = 2.5;
  public static final double ELEVATOR_S = 0.39448793854367;
  public static final double ELEVATOR_V = 1.027996641809018;
  public static final double ELEVATOR_A = 0.057759439794094;
  public static final double ELEVATOR_G = 0.35;

  public static final double DRUM_RADIUS = 1;
  public static final double ELEVATOR_GEAR_RATIO = 8;
  public static final double ELEVATOR_CARRIAGE_MASS = 10;
  public static final double MIN_HEIGHT = 0;
  public static final double MAX_HEIGHT = 3;
  public static final double INCLINE_ANGLE_RADIANS = Units.degreesToRadians(8);
  public static final boolean SIMULATE_GRAVITY = true;

  public static final double STATOR_CURRENT_LIMIT = 100;
  public static final double SUPPLY_CURRENT_LIMIT = 0;
  public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = false;

  public static final double MOTION_MAGIC_MAX_ACCELERATION = 160;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 100;

  public static final double ELEVATOR_ERROR_TOLERANCE = 0.08;

  // limit
  public static final double LIMIT = -0.01;
  public static final boolean LIMIT_ENABLE = false;
  public static final double REVERSE_LIMIT = -9.6;
  public static final boolean REVRESE_LIMIT_ENABLE = true;

  // Elevator setpoints
  public enum ElevatorSetpoints {
    L1(-2.4),
    L2(-3.65),
    L3(-5.845),
    L4(-9.5),
    FEEDER(-1.2646484375);

    private final double position;

    ElevatorSetpoints(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
