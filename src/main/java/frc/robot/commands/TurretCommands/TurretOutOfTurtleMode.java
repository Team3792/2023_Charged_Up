// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretOutOfTurtleMode extends CommandBase {
  /** Creates a new ToTurtleMode. */
  TurretSubsystem turretSubsystem;
Timer timer = new Timer();
  public TurretOutOfTurtleMode(TurretSubsystem turretSubsystem) {

    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    turretSubsystem.doneMoving = false;
    turretSubsystem.addOn = 180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.setPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("endingggggg");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretSubsystem.doneMoving||timer.hasElapsed(5);
  }
}
