package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.utils.LookupTable;

public class LookupTableContents {
  // todo tune

  /**
   * The aiming lookup table. Key is distance in inches, value1 is RPM, value2 is angle value
   */
  public static final LookupTable.Entry[] TABLE_CONTENTS = {
    new LookupTable.Entry(0.0, 0.0, 0.0),
    new LookupTable.Entry(1.0, 1.0, 1.0),
    new LookupTable.Entry(2.0, 2.0, 2.0),
    new LookupTable.Entry(3.0, 3.0, 3.0),
    new LookupTable.Entry(4.0, 4.0, 4.0),
    new LookupTable.Entry(5.0, 5.0, 5.0),
    new LookupTable.Entry(6.0, 6.0, 6.0),
    new LookupTable.Entry(7.0, 7.0, 7.0),
    new LookupTable.Entry(8.0, 8.0, 8.0),
    new LookupTable.Entry(9.0, 9.0, 9.0),
    new LookupTable.Entry(10.0, 10.0, 10.0)
  };
}
