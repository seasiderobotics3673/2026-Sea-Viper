// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoWait extends Command {

 private long destinationTime;
 private long delay;

  /** Creates a new AutoWait. */
  public AutoWait(long delay, BooleanSupplier abortFlag) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.delay = delay;
  }

  public AutoWait(long delay) {
    this(delay, () -> false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    destinationTime = System.currentTimeMillis() + delay;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(System.currentTimeMillis());
    //System.out.println(destinationTime);
  }

  public void abort() {
    destinationTime = System.currentTimeMillis();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long curTime = System.currentTimeMillis();
    if (curTime > destinationTime) {
      return true;
    } else {
      //System.out.println(curTime);
      return false;
    }
  }
}
