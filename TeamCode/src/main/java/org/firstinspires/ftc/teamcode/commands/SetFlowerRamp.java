/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlowerRamp;

import java.util.Set;

/**
 * Command to set the state of the flower intake ramp
 */
public class SetFlowerRamp implements Command {
  private final FlowerRamp ramp;
  private final FlowerRamp.State state;

  public SetFlowerRamp(FlowerRamp ramp, FlowerRamp.State state) {
    this.ramp = ramp;
    this.state = state;
  }

  @Override
  public Set<Object> requirements() {
    return Set.of(ramp);
  }

  @Override
  public int priority() {
    return 8;
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
    ramp.setState(state);
  }

  @Override
  public boolean done() {
    return ramp.atTarget();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(EndCondition endCondition) {

  }
}
