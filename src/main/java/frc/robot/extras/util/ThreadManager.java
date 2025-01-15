package frc.robot.extras.util;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

/**
 * Manages threads for various tasks within the robot code. Provides methods to start, stop, and
 * manage threads efficiently while preventing excessive updates.
 *
 * @author Ishan
 */
public class ThreadManager {

  private final ExecutorService executorService;
  private final Map<String, Future<?>> threadFutures = new ConcurrentHashMap<>();

  /**
   * Creates a ThreadManager with a fixed number of threads.
   *
   * @param maxThreads the maximum number of threads that can run simultaneously.
   */
  public ThreadManager(int maxThreads) {
    executorService = Executors.newFixedThreadPool(maxThreads);
  }

  /**
   * Starts a new thread to run a specific task. If a thread with the given name is already running,
   * the method will print an error message and do nothing.
   *
   * @param threadName the name of the thread to start.
   * @param task the task to run on the thread, this can be a function.
   */
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

  /**
   * Stops a specific thread by its name. If the thread is not running, an error message is printed.
   *
   * @param threadName the name of the thread to stop.
   */
  public void stopThread(String threadName) {
    Future<?> future = threadFutures.get(threadName);
    if (future != null) {
      future.cancel(true);
      threadFutures.remove(threadName);
    } else {
      System.err.println("Thread " + threadName + " is not running.");
    }
  }

  /**
   * Starts a task that periodically updates and will sleep for the specified sleepTime. This method
   * creates a thread that repeatedly executes the provided task until interrupted.
   *
   * @param threadName the name of the thread to start.
   * @param task the task that is to be run.
   * @param sleepTime the time to sleep between updates in milliseconds.
   */
  public void startTask(String threadName, Runnable task, long sleepTime) {
    startThread(
        threadName,
        () -> {
          try {
            while (!Thread.currentThread().isInterrupted()) {
              task.run();
              Thread.sleep(sleepTime); // Sleep to avoid excessive updates
            }
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }
        });
  }

  /**
   * Shuts down all threads managed by this instance. Attempts a graceful shutdown, then forces it
   * if necessary.
   */
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
