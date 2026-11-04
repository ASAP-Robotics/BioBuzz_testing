/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.MathUtils.areEqual;
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
      if (Double.isNaN(key) ||
          Double.isInfinite(key) ||
          Double.isNaN(value1) ||
          Double.isNaN(value2)) {
        throw new IllegalArgumentException("Key and values must be valid, and key must be finite");
      }

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
   *             entries with invalid values or duplicate, infinite, or missing keys
   * @implNote this isn't the most efficient it could be, so don't use after init
   */
  public LookupTable(Entry[] list) {
    // validate list to ensure at least two entries
    if (list == null || list.length < 2) {
      throw new IllegalArgumentException("List must not be null, and contain at least two entries");
    }

    // validate list to ensure unique, valid keys
    for (int i = 0; i < list.length - 1; i++) {
      if (list[i] == null) {
        throw new IllegalArgumentException("List entries must not be null");
      }
      for (int j = i + 1; j < list.length; j++) {
        if (areEqual(list[i].key, list[j].key)) {
          throw new IllegalArgumentException("List must not contain entries with duplicate keys");
        }
      }
    }

    this.list = list.clone();

    // sort list from low to high by keys
    Arrays.sort(this.list, Comparator.comparingDouble(t0 -> t0.key));
  }

  /**
   * Finds the first entry in the lookup table with exactly the provided key value
   * @param key the key to search for
   * @return the first entry in the table with the provided key, or null if none are found
   */
  public Entry getKeyExact(double key) {
    int idx =
        Arrays.binarySearch(
            list,
            new Entry(key, 0.0, 0.0),
            Comparator.comparingDouble(t0 -> t0.key)
        );

    if (idx >= 0) {
      return list[idx];

    } else {
      return null;
    }
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
    Entry above; // the entry "above" the requested one
    Entry below; // the entry "below" the requested one

    int idx =
        Arrays.binarySearch(
            list,
            new Entry(key, 0.0, 0.0),
            Comparator.comparingDouble(t0 -> t0.key)
        );

    if (idx >= 0) { // key exactly in table
      return list[idx];

    } else if (idx == -1) { // key below all in table
      above = list[1];
      below = list[0];

    } else if (idx == -(list.length + 1)) { // key above all in table
      above = list[list.length - 1];
      below = list[list.length - 2];

    } else { // key between two in table
      above = list[-idx - 1]; // above = insertion index
      below = list[-idx - 2]; // below = 1 below insertion index
    }

    // get values corresponding to requested key
    double value1 = map(key, below.key, above.key, below.value1, above.value1);
    double value2 = map(key, below.key, above.key, below.value2, above.value2);

    return new Entry(key, value1, value2);
  }
}
