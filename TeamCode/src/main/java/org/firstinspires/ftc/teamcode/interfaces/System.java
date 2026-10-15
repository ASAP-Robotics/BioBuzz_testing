/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.interfaces;

/** Interface for systems */
public interface System {
  /** to be called once, on program initialization */
  void init();

  /** to be called each init loop */
  void initLoop();

  /** to be called once, on program start */
  void start();

  /** to be called each program loop */
  void loop();

  /** to be called once, on program end */
  void stop();
}
