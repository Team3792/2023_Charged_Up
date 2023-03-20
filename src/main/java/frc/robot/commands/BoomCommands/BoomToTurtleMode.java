// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BoomCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BoomSubsystem;

public class BoomToTurtleMode extends CommandBase {
  /** Creates a new BoomTurtleMode. */
  BoomSubsystem boomSubsystem;

  public BoomToTurtleMode(BoomSubsystem boomSubsystem) {
    this.boomSubsystem = boomSubsystem;
    addRequirements(boomSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boomSubsystem.doneMoving = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boomSubsystem.setPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return boomSubsystem.doneMoving;
  }
}
