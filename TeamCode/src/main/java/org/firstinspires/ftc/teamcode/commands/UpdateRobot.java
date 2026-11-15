package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.configurations.RobotHardware;

import java.util.Collections;
import java.util.Set;

/**
 * Command to call the loop() methods of the robot hardware code
 */
public class UpdateRobot implements Command {
  private final RobotHardware robot;

  public UpdateRobot(RobotHardware robot) {
    this.robot = robot;
  }

  @Override
  public Set<Object> requirements() {
    return Collections.emptySet(); // maybe change? IDK, probably doesn't need it
  }

  @Override
  public int priority() {
    return 10; // maybe make nicer?
  }

  @Override
  public InterruptedBehavior interruptedBehavior() {
    return InterruptedBehavior.SUSPEND;
  }

  @Override
  public ConflictBehavior conflictBehavior() {
    return ConflictBehavior.QUEUE;
  }

  @Override
  public BlockedBehavior blockedBehavior() {
    return BlockedBehavior.QUEUE;
  }

  @Override
  public void start() {

  }

  @Override
  public boolean done() {
    return false;
  }

  @Override
  public void execute() {
    robot.scoringSystem.loop();
  }

  @Override
  public void end(EndCondition endCondition) {

  }
}
