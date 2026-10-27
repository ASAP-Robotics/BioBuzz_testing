/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.interfaces;

/** Interface for systems */
public interface System {
  /** To be called once, on program initialization */
  void init();

  /** To be called each init loop */
  void initLoop();

  /** To be called once, on program start */
  void start();

  /** To be called each program loop */
  void loop();

  /** To be called once, on program end */
  void stop();
}
