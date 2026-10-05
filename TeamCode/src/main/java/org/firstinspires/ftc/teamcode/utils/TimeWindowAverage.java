/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * Class to take the average of a number of readings over a set time window
 */
public class TimeWindowAverage {
    /**
     * Class to hold a reading of some sort to be averaged
     */
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

    /**
     * Class to hold a calculated average value
     */
    public static class Average {
        /** How confident the reading is, from 0.0 - 1.0 */
        public final double confidence;
        /** The value of the average */
        public final double average;

        public Average(double confidence, double average) {
            this.confidence = confidence;
            this.average = average;
        }

        /**
         * Gets if the average is fully confident
         *
         * @return true if the average is confident, false if it was based on too few readings
         */
        public boolean confident() {
            return this.confidence >= 1.0;
        }
    }

    private final Queue<Reading> list;
    private long windowSpan;
    private int minReadings;

    /**
     * Makes a new TimeWindowAverage
     *
     * @param windowSpan the timespan to average over
     * @param minReadings the minimum number of readings to base a confident average off of
     */
    public TimeWindowAverage(long windowSpan, int minReadings) {
        this.windowSpan = windowSpan;
        this.minReadings = minReadings;
        this.list = new ArrayDeque<>();
    }

    /**
     * Adds a new reading to the list of readings to average
     *
     * @param data the value of the reading
     */
    public void addReading(double data) throws IllegalArgumentException {
        if (Double.isNaN(data) || Double.isInfinite(data)) throw new IllegalArgumentException();
        list.add(new Reading(data));
    }

    /**
     * Adds a new reading to the list of readings to average
     *
     * @param reading the reading to add
     * @apiNote this is included in case it's useful, but prefer the other overload
     * @implNote this method doesn't validate the reading timestamps passed to it, use caution
     */
    public void addReading(Reading reading) {
        if (reading == null || Double.isNaN(reading.data) || Double.isInfinite(reading.data))
            throw new IllegalArgumentException();
        list.add(reading);
    }

    /**
     * Gets the average value of all readings within the last average window
     *
     * @return the average of the recorded readings
     */
    public Average getAverage() throws IllegalStateException {
        cleanList();

        if (list.isEmpty()) throw new IllegalStateException("List must not be empty");

        int size = list.size();
        double sum = 0;

        for (Reading i : list) {
            sum += i.data;
        }

        return new Average(size >= minReadings ? 1.0 : (double) size / minReadings, sum / size);
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

    /**
     * Cleans up the list by removing old readings
     */
    private void cleanList() {
        long now = Timestamp.now();

        while (!list.isEmpty() && list.peek().timeStamp + windowSpan <= now) {
            list.remove();
        }
    }
}
