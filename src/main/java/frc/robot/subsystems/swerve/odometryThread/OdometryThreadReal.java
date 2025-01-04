// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package frc.robot.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThreadReal extends Thread implements OdometryThread {
  DeviceCANBus canBus;

  private final OdometryDoubleInput[] odometryDoubleInputs;
  private final BaseStatusSignal[] statusSignals;
  private final Queue<Double> timeStampsQueue;
  private final Lock lock = new ReentrantLock();

  public OdometryThreadReal(
      DeviceCANBus canBus,
      OdometryDoubleInput[] odometryDoubleInputs,
      BaseStatusSignal[] statusSignals) {
    this.canBus = canBus;
    // Create a queue with a capacity of 10
    this.timeStampsQueue = new ArrayBlockingQueue<>(10);
    this.odometryDoubleInputs = odometryDoubleInputs;
    this.statusSignals = statusSignals;

    setName("OdometryThread");
    setDaemon(true);
  }

  @Override
  public synchronized void start() {
    if (odometryDoubleInputs.length > 0) super.start();
  }

  @Override
  public void run() {
    while (true) odometryPeriodic();
  }

  private void odometryPeriodic() {
    refreshSignalsAndBlockThread();

    lock.lock();
    timeStampsQueue.offer(estimateAverageTimeStamps());
    for (OdometryDoubleInput odometryDoubleInput : odometryDoubleInputs)
      odometryDoubleInput.cacheInputToQueue();
    lock.unlock();
  }

  private void refreshSignalsAndBlockThread() {
    switch (canBus) {
      case RIO -> {
        BaseStatusSignal.setUpdateFrequencyForAll(
            1.0 / HardwareConstants.SIGNAL_FREQUENCY, statusSignals);
        BaseStatusSignal.refreshAll();
      }
      case CANIVORE -> BaseStatusSignal.waitForAll(HardwareConstants.TIMEOUT_S, statusSignals);
    }
  }

  private double estimateAverageTimeStamps() {
    double currentTime = TimeUtil.getRealTimeSeconds(), totalLatency = 0;
    for (BaseStatusSignal signal : statusSignals)
      totalLatency += signal.getTimestamp().getLatency();

    if (statusSignals.length == 0) return currentTime;
    return currentTime - totalLatency / statusSignals.length;
  }

  @Override
  public void updateInputs(OdometryThreadInputs inputs) {
    inputs.measurementTimeStamps = new double[timeStampsQueue.size()];
    for (int i = 0; i < inputs.measurementTimeStamps.length && !timeStampsQueue.isEmpty(); i++)
      inputs.measurementTimeStamps[i] = timeStampsQueue.poll();
  }

  @Override
  public void lockOdometry() {
    lock.lock();
  }

  @Override
  public void unlockOdometry() {
    lock.unlock();
  }
}
