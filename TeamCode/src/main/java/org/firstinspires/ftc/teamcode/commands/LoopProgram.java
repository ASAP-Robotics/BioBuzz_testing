/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import java.util.Collections;
import java.util.Set;
import org.firstinspires.ftc.teamcode.opmodetypes.CommonProgram;

/** Command to call the loop() method of a program */
@Deprecated
public class LoopProgram implements Command {
  private final CommonProgram program;

  public LoopProgram(CommonProgram program) {
    this.program = program;
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
  public void start() {}

  @Override
  public boolean done() {
    return false; // never done
  }

  @Override
  public void execute() {
    program.loop();
  }

  @Override
  public void end(EndCondition endCondition) {}
}
