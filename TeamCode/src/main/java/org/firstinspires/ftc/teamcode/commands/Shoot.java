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
 * Command to shoot the robot's payload
 */
public class Shoot implements Command {
  private final RobotHardware robot;

  public Shoot(RobotHardware robot) {
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
    robot.scoringSystem.shoot();
  }

  @Override
  public boolean done() {
    ScoringSystem.State state = robot.scoringSystem.getState();
    return state != ScoringSystem.State.SHOT_QUEUED && state != ScoringSystem.State.SHOOTING;
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(EndCondition endCondition) {

  }
}
