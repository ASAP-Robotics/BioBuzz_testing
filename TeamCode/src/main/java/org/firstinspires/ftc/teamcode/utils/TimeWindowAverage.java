/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

/** Class to take the average of a number of readings over a set time window */
public class TimeWindowAverage {
  /** Class to hold a reading of some sort to be averaged */
  public static class Reading {
    public final double data;
    public final long timeStamp;

    public Reading(double data, long timeStamp) {
      this.data = data;
      this.timeStamp = timeStamp;
    }

    public Reading(double data) {
      this.data = data;
      this.timeStamp = Timestamp.now();
    }
  }

  /** Class to hold a calculated average value */
  public static class Average {
    /** How confident the reading is, from 0.0 - 1.0 (-1 is an invalid reading) */
    public final double confidence;

    /** The number of readings the average is based off of */
    public final int readingNumber;

    /** The value of the average */
    public final double average;

    public Average(double confidence, int readingNumber, double average) {
      this.confidence = confidence;
      this.readingNumber = readingNumber;
      this.average = average;
    }

    /**
     * Gets if the average is fully confident
     *
     * @return true if the average is confident, false if it was based on too few readings
     * @apiNote this is guaranteed to return false if the average is invalid
     */
    public boolean confident() {
      return this.confidence >= 1.0;
    }

    /**
     * Gets if the average is valid or usable
     *
     * @return true is usable, false if unusable
     */
    public boolean valid() {
      return this.confidence >= 0;
    }
  }

  /**
   * How often the list should be cleaned automatically to same memory, in milliseconds. Made
   * slightly weird so cleaning doesn't coincide with other time-based "slow" tasks
   */
  private static final double AUTO_CLEAN_LIST_INTERVAL_MS = 5377;

  /** List (Deque) of readings */
  private final Deque<Reading> list;

  /** Timer to track how long it's been since the list has been cleaned */
  private final ElapsedTime timeSinceLastClean;

  /** A running sum of the readings in the list */
  private double runningSum = 0.0;

  /** The size of the time window to take averages over, in milliseconds */
  private long windowSpan;

  /** The minimum number of readings for a confident average to be generated from */
  private int minReadings;

  /**
   * Makes a new TimeWindowAverage
   *
   * @param windowSpan the timespan to average over (milliseconds)
   * @param minReadings the minimum number of readings to base a confident average off of
   */
  public TimeWindowAverage(long windowSpan, int minReadings) throws IllegalArgumentException {
    if (windowSpan <= 0.0 || minReadings <= 0) {
      throw new IllegalArgumentException("Window span and min readings must be positive");
    }
    this.windowSpan = windowSpan;
    this.minReadings = minReadings;
    this.list = new ArrayDeque<>();
    this.timeSinceLastClean = new ElapsedTime();
  }

  /**
   * Adds a new reading to the list of readings to average
   *
   * @param data the value of the reading
   */
  public void addReading(double data) throws IllegalArgumentException {
    if (Double.isNaN(data) || Double.isInfinite(data)) throw new IllegalArgumentException();
    list.add(new Reading(data));
    runningSum += data;

    if (timeSinceLastClean.milliseconds() >= AUTO_CLEAN_LIST_INTERVAL_MS) {
      cleanList();
    }
  }

  /**
   * Adds a new reading to the list of readings to average
   *
   * @param reading the reading to add
   * @apiNote this is included in case it's useful, but prefer the other overload
   * @implNote this method doesn't fully validate the reading passed to it, use caution
   */
  public void addReading(Reading reading) throws IllegalArgumentException {
    // this shouldn't ever throw a NullPointerException, ignore warning
    if (reading == null
        || Double.isNaN(reading.data)
        || Double.isInfinite(reading.data)
        || (!list.isEmpty() && reading.timeStamp < list.peekLast().timeStamp))
      throw new IllegalArgumentException();
    list.add(reading);
    runningSum += reading.data;

    if (timeSinceLastClean.milliseconds() >= AUTO_CLEAN_LIST_INTERVAL_MS) {
      cleanList();
    }
  }

  /** Clears all past readings */
  public void clearAverage() {
    list.clear();
    runningSum = 0.0;
    timeSinceLastClean.reset();
  }

  /**
   * Gets the average value of all readings within the last average window
   *
   * @return the average of the recorded readings
   */
  public Average getAverage() {
    cleanList();

    if (list.isEmpty()) {
      return new Average(-1.0, 0, 0.0);
    }

    int readingNumber = list.size();
    double confidence = readingNumber >= minReadings ? 1.0 : (double) readingNumber / minReadings;

    return new Average(confidence, readingNumber, runningSum / readingNumber);
  }

  /**
   * Gets the most recently added reading
   *
   * @return the latest reading
   * @apiNote this doesn't ensure that the returned reading is inside the averaging window
   */
  public Reading getLatestReading() {
    return list.peekLast();
  }

  /**
   * Gets an unmodifiable copy of the internal list of readings
   *
   * @return an unmodifiable list of readings
   * @apiNote this method shouldn't be used; it copies the entire internal list, which is slow
   */
  @Deprecated
  public List<Reading> getReadingList() {
    cleanList();
    return Collections.unmodifiableList(new ArrayList<>(list));
  }

  /**
   * Gets the difference between the data of the readings with the highest and lowest data values
   *
   * @return the difference between the data of the readings with the highest and lowest data values
   */
  public double getMaxMinReadingDifference() {
    cleanList();

    if (list.isEmpty()) return 0.0;

    double maxData = Double.NEGATIVE_INFINITY;
    double minData = Double.POSITIVE_INFINITY;

    for (Reading i : list) {
      double readingData = i.data;
      if (readingData > maxData) maxData = readingData;
      if (readingData < minData) minData = readingData;
    }

    return maxData - minData;
  }

  /**
   * Gets the n readings with data closest to a given value
   *
   * @param n the number of readings to return
   * @param compareTo the data value the returned values will be close to (should be finite)
   * @return an array of the n closest readings to the given value
   */
  public Reading[] getClosestNReadings(int n, double compareTo) throws IllegalArgumentException {
    cleanList();
    // allow `n` values of 0, to match empty list behavior
    if (n < 0 || Double.isNaN(compareTo) || Double.isInfinite(compareTo)) {
      throw new IllegalArgumentException();
    }
    n = Math.min(n, list.size()); // clamp n to below list size to avoid unnecessary exceptions

    Reading[] closest = new Reading[n];

    // go through list to find closest n readings
    int fillCount = 0; // using counter instead of normal FOR loop for performance
    int worstIndex = -1;
    double worstError = Double.NEGATIVE_INFINITY;
    for (Reading i : list) {
      // check if list of closest readings isn't full yet
      if (fillCount < closest.length) {
        closest[fillCount++] = i;
      } else {
        // if we need to recalculate furthest value / index in the list of closest readings
        if (worstIndex == -1) {
          // go through list of closest readings to find furthest reading in it
          for (int j = 0; j < closest.length; j++) {
            double readingErrorJ = Math.abs(compareTo - closest[j].data);
            if (readingErrorJ > worstError) {
              worstError = readingErrorJ;
              worstIndex = j;
            }
          }
        }

        // check if reading is closer than the furthest reading in current list of closest readings
        double readingErrorI = Math.abs(compareTo - i.data);
        if (readingErrorI < worstError) {
          closest[worstIndex] = i;
          worstIndex = -1; // mark for recalculation
          worstError = Double.NEGATIVE_INFINITY; // ensure worst error is recalculated
        }
      }
    }

    return closest;
  }

  /**
   * Gets the number of readings within the average window span
   *
   * @return the number of readings within the average window span
   */
  public int getNumReadings() {
    cleanList();
    return list.size();
  }

  /**
   * Sets the duration of the averaging window
   *
   * @param windowSpan the duration of the averaging window, in milliseconds
   */
  public void setWindowSpan(long windowSpan) throws IllegalArgumentException {
    if (windowSpan <= 0) throw new IllegalArgumentException();
    this.windowSpan = windowSpan;
  }

  /**
   * Sets the number of readings required for an average to be confident
   *
   * @param minReadings the minimum number of readings to base a confident average off of
   */
  public void setMinReadings(int minReadings) throws IllegalArgumentException {
    if (minReadings <= 0) throw new IllegalArgumentException();
    this.minReadings = minReadings;
  }

  /** Cleans up the list by removing old readings */
  private void cleanList() {
    long now = Timestamp.now();

    // this shouldn't ever throw an exception, ignore warning
    while (!list.isEmpty() && now - list.peek().timeStamp > windowSpan) {
      runningSum -= list.remove().data;
    }

    timeSinceLastClean.reset();
  }
}
