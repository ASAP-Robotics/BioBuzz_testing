/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.interfaces.System;

import java.util.List;

/**
 * Class to enable and handle bulk sensor read caching for OpModes
 */
public class BulkCachingHelper implements System {
  protected List<LynxModule> allHubs;

  public BulkCachingHelper(HardwareMap hwMap) {
    this.allHubs = hwMap.getAll(LynxModule.class);

    // enable manual bulk caching for all Lynx module hubs
    for (LynxModule hub : this.allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
  }

  @Override
  public void init() {
    clearSensorCache();
  }

  @Override
  public void initLoop() {
    clearSensorCache();
  }

  @Override
  public void start() {
    clearSensorCache();
  }

  @Override
  public void loop() {
    clearSensorCache();
  }

  @Override
  public void stop() {
    clearSensorCache();
  }

  /**
   * Clears the bulk reading sensor caches for all Lynx module hubs
   */
  public void clearSensorCache() {
    for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
    }
  }
}
