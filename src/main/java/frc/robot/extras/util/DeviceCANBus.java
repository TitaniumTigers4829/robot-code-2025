package frc.robot.extras.util;

import frc.robot.Constants.HardwareConstants;

/** CTRE Phoenix CAN bus */
public enum DeviceCANBus {
  /** roboRIO CAN bus */
  RIO(HardwareConstants.RIO_CAN_BUS_STRING),
  /**
   * CANivore CAN bus
   *
   * <p>Only a single CANivore is supported, and MUST be named "canivore"
   */
  CANIVORE(HardwareConstants.CANIVORE_CAN_BUS_STRING);

  /** CAN bus name */
  public final String name;

  private DeviceCANBus(String name) {
    this.name = name;
  }
}
