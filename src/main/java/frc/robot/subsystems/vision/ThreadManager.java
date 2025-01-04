package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionInterface.VisionInputs;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class ThreadManager {
  private final ExecutorService executorService;
  private final Map<String, Future<?>> threadFutures = new ConcurrentHashMap<>();

  public ThreadManager(int maxThreads) {
    executorService = Executors.newFixedThreadPool(maxThreads);
  }

  // Start a thread for a specific task
  public void startThread(String threadName, Runnable task) {
    if (threadFutures.containsKey(threadName)) {
      System.err.println("Thread " + threadName + " is already running.");
      return;
    }

    Future<?> future =
        executorService.submit(
            () -> {
              try {
                task.run();
              } catch (Exception e) {
                System.err.println("Error in thread " + threadName + ": " + e.getMessage());
              }
            });

    threadFutures.put(threadName, future);
  }

  // Stop a specific thread
  public void stopThread(String threadName) {
    Future<?> future = threadFutures.get(threadName);
    if (future != null) {
      future.cancel(true);
      threadFutures.remove(threadName);
    } else {
      System.err.println("Thread " + threadName + " is not running.");
    }
  }

  // Submit a vision input task
  public void startVisionInputTask(String threadName, VisionInputs inputs, Runnable visionTask) {
    startThread(
        threadName,
        () -> {
          try {
            while (!Thread.currentThread().isInterrupted()) {
              visionTask.run();
              Thread.sleep(VisionConstants.THREAD_SLEEP_MS); // Sleep to avoid excessive updates
            }
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }
        });
  }

  // Shut down all threads
  public void shutdownAllThreads() {
    threadFutures.values().forEach(future -> future.cancel(true));
    threadFutures.clear();
    executorService.shutdown();
    try {
      if (!executorService.awaitTermination(5, TimeUnit.SECONDS)) {
        executorService.shutdownNow();
      }
    } catch (InterruptedException e) {
      executorService.shutdownNow();
      Thread.currentThread().interrupt();
    }
  }
}
