// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int ELEVATOR_LEADER_MOTOR_ID = 0;
  public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 62;

  public static final double ELEVATOR_P = 1;
  public static final double ELEVATOR_I = 0;
  public static final double ELEVATOR_D = 0;
  public static final double ELEVATOR_S = 0;
  public static final double ELEVATOR_V = 0;
  public static final double ELEVATOR_A = 0;
  public static final double ELEVATOR_G = 0;

  public static final double DRUM_RADIUS = 1;
  public static final double ELEVATOR_GEAR_RATIO = 8;
  public static final double ELEVATOR_CARRIAGE_MASS = 10;
  public static final double MIN_HEIGHT = 0;
  public static final double MAX_HEIGHT = 3;
  public static final double INCLINE_ANGLE_RADIANS = Units.degreesToRadians(8);
  public static final boolean SIMULATE_GRAVITY = true;

  public static final double STATOR_CURRENT_LIMIT = 0;
  public static final double SUPPLY_CURRENT_LIMIT = 0;
  public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = false;

  public static final double MOTION_MAGIC_MAX_ACCELERATION = 2;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2;

  // limit
  public static final double LIMIT = 40;
  public static final boolean LIMIT_ENABLE = false;
  public static final double REVERSE_LIMIT = 0.15;
  public static final boolean REVRESE_LIMIT_ENABLE = false;

  public static final double ELEVATOR_FEEDING_STATION_HEIGHT = 0.0;
}
