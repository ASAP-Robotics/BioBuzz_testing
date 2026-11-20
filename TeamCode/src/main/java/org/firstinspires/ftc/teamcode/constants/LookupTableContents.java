package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.utils.LookupTable;

public class LookupTableContents {
  // todo tune better

  /**
   * The aiming lookup table. Key is distance in inches, value1 is RPM, value2 is angle value
   * (angle is unused)
   */
  public static final LookupTable.Entry[] TABLE_CONTENTS = {
    new LookupTable.Entry(46.0, 3400, 0.0),
    new LookupTable.Entry(34, 3100, 0.0),
    new LookupTable.Entry(22, 2900, 0.0)
  };
}
