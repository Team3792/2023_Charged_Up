// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BoomSubsystem;

public class ExtendBoomAuto extends CommandBase {
  /** Creates a new ExtendBoomAuto. */
  BoomSubsystem boomSubsystem;
  String piece;

  public ExtendBoomAuto(BoomSubsystem boomSubsystem, String piece) {
    this.boomSubsystem = boomSubsystem;
    this.piece = piece;

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
    if(piece == "cube"){
    boomSubsystem.setPosition(Constants.BoomConstants.kAutoCubeReach);
    }else {
    boomSubsystem.setPosition(Constants.BoomConstants.kAutoConeReach);
    }
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
