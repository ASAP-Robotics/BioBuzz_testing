/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.configurations.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.ScoringSystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command to mark the robot's intake as full
 */
public class SetFull implements Command {
  private final RobotHardware robot;

  public SetFull(RobotHardware robot) {
    this.robot = robot;
  }

  @Override
  public Set<Object> requirements() {
    return Collections.emptySet();
  }

  @Override
  public int priority() {
    return 9;
  }

  @Override
  public InterruptedBehavior interruptedBehavior() {
    return InterruptedBehavior.END;
  }

  @Override
  public ConflictBehavior conflictBehavior() {
    return ConflictBehavior.OVERRIDE;
  }

  @Override
  public BlockedBehavior blockedBehavior() {
    return BlockedBehavior.QUEUE;
  }

  @Override
  public void start() {
    robot.scoringSystem.setFull();
  }

  @Override
  public boolean done() {
    return true;
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(EndCondition endCondition) {

  }
}
