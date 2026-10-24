/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.TestOnly;

/**
 * Class to track loop times in an OpMode
 *
 * @apiNote this class assumes that Panels telemetry is updated regularly elsewhere
 */
@Configurable
public class LoopTimeTracker {
  /** If telemetry should be sent directly from this class */
  public static boolean TELEMETRY_ENABLE = false;
  /** How often telemetry should be updated, in milliseconds */
  public static double TELEMETRY_UPDATE_INTERVAL = 1000;

  /** The time to measure loop time averages over, in milliseconds */
  private static final long AVERAGE_WINDOW_MS = 1000;
  /** Timer for measuring loop times */
  private final ElapsedTime loopTimer;
  private final ElapsedTime telemetryUpdateTimer;
  /** Time window average for loop times */
  private final TimeWindowAverage loopTimeAverage;
  /** Telemetry to send data to */
  private final TelemetryManager tlm;

  public LoopTimeTracker() {
    this.loopTimer = new ElapsedTime();
    this.telemetryUpdateTimer = new ElapsedTime();
    this.loopTimeAverage = new TimeWindowAverage(AVERAGE_WINDOW_MS, 1);
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  /**
   * Resets the timer of how long it's been since the last loop
   */
  public void start() {
    loopTimer.reset();
    telemetryUpdateTimer.reset();
  }

  /**
   * Records a new loop, and if telemetry is enabled, updates telemetry
   */
  public void loop() {
    double lastLoopTime = loopTimer.milliseconds();
    loopTimer.reset();
    loopTimeAverage.addReading(lastLoopTime);

    if (telemetryUpdateTimer.milliseconds() >= TELEMETRY_UPDATE_INTERVAL) {
      updateTelemetry();
      telemetryUpdateTimer.reset();
    }
  }

  /**
   * Gets the duration of the latest recorded loop
   *
   * @return the length of the most recent loop, in milliseconds
   * @apiNote this method is for live telemetry graphing, but isn't meant for competition code
   */
  @TestOnly
  public double getLatestLoopTime() {
    return loopTimeAverage.getLatestReading().data;
  }

  /**
   * Gets the average loop time
   *
   * @return the time of the average loop, in milliseconds
   */
  public double getAverageLoopTimeMs() {
    return loopTimeAverage.getAverage().average;
  }

  /**
   * Gets the jitter, or difference between min and max values, of loop times
   *
   * @return the loop time jitter, in milliseconds
   */
  public double getLoopTimeJitterMs() {
    return loopTimeAverage.getMaxMinReadingDifference();
  }

  /**
   * Gets the average loop time of the loops with the highest 5% of loop times
   *
   * @return the average time of the highest 5% of loops, in milliseconds
   */
  public double getLoopTime5PercentHighsMs() {
    int numTopTimes = (int) Math.ceil(loopTimeAverage.getNumReadings() * 0.05);
    double topLoopTimesSum = 0.0; // on an empty list, this should be returned

    for (TimeWindowAverage.Reading i :
        loopTimeAverage.getClosestNReadings(numTopTimes, Double.MAX_VALUE)) {
      topLoopTimesSum += i.data;
    }

    return topLoopTimesSum / numTopTimes;
  }

  /**
   * Gets the average loop time of the loops with the lowest 5% of loop times
   *
   * @return the average time of the lowest 5% of loops, in milliseconds
   */
  public double getLoopTime5PercentLowsMs() {
    int numTopTimes = (int) Math.ceil(loopTimeAverage.getNumReadings() * 0.05);
    double topLoopTimesSum = 0.0; // on an empty list, this should be returned

    for (TimeWindowAverage.Reading i :
        loopTimeAverage.getClosestNReadings(numTopTimes, -Double.MAX_VALUE)) {
      topLoopTimesSum += i.data;
    }

    return topLoopTimesSum / numTopTimes;
  }

  /**
   * Updates the telemetry from this class
   */
  private void updateTelemetry() {
    tlm.addData("Loop Time Average (ms)", getAverageLoopTimeMs());
    tlm.addData("Loop Time Jitter (ms)", getLoopTimeJitterMs());
    tlm.addData("Loop Time 5% Highs (ms)", getLoopTime5PercentHighsMs());
    tlm.addData("Loop Time 5% Lows (ms)", getLoopTime5PercentLowsMs());
  }
}
