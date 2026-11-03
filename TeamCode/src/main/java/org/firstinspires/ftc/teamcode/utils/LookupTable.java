/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.MathUtils.map;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Simple class to implement a lookup table with two values per entry
 */
public class LookupTable {
  public static class Entry {
    public final double key;
    public final double value1;
    public final double value2;

    public Entry(double key, double value1, double value2) {
      this.key = key;
      this.value1 = value1;
      this.value2 = value2;
    }
  }

  /** The list of lookup table items */
  public final Entry[] list;

  /**
   * Makes a new `LookupTable`
   *
   * @param list an array of table entries; must contain at least two entries and must not contain
   *             entries with duplicate keys
   */
  public LookupTable(Entry[] list) {
    // validate list to ensure at least two entries
    if (list.length < 2) {
      throw new IllegalArgumentException("List must contain at least two entries");
    }

    // validate list to ensure unique keys
    for (int i = 0; i < list.length - 1; i++) {
      for (int j = i + 1; j < list.length; j++) {
        if (list[i].key == list[j].key) {
          throw new IllegalArgumentException("List must not contain entries with duplicate keys");
        }
      }
    }

    // sort list from low to high by keys
    Arrays.sort(list, Comparator.comparingDouble(t0 -> t0.key));
    this.list = list;
  }

  /**
   * Finds the first entry in the lookup table with the exact provided key value
   * @param key the key to search for
   * @return the first entry in the table with the provided key, or null if none are found
   */
  public Entry getKeyExact(double key) {
    for (Entry i : list) {
      if (i.key == key) {
        return i;
      }
    }

    return null;
  }

  /**
   * Makes a new "virtual" entry, interpolating between (or extrapolating from) the two entries with
   * keys closest to the one provided
   *
   * @param key the key value to use when interpolating
   * @return an Entry created from interpolating between the two closest table entries
   * @implNote assumes that `list` is sorted low to high by keys, and has at least 2 entries
   */
  public Entry getKeyInterpolate(double key) {
    Entry above = null; // the entry "above" the requested one
    Entry below = null; // the entry "below" the requested one

    // check for requested key being between two in table or below any in table
    for (int i = 0; i < list.length; i++) {
      if (list[i].key > key) { // first entry with key greater than requested
        if (i == 0) { // requested key is below any in table
          above = list[1];
          below = list[0];
        } else { // requested key is between two in table
          above = list[i];
          below = list[i - 1];
        }
        break;
      }
    }

    // check for requested key being above any in table
    if (above == null || below == null) { // requested key is above any in table
      above = list[list.length - 1];
      below = list[list.length - 2];
    }

    // get values corresponding to requested key
    double value1 = map(key, below.key, above.key, below.value1, above.value1);
    double value2 = map(key, below.key, above.key, below.value2, above.value2);

    return new Entry(key, value1, value2);
  }
}
