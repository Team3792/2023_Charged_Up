// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakePreparationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class LowIntakeConePreparation extends CommandBase {
  /** Creates a new HighIntakePreperation. */

 
  ElevatorSubsystem elevatorSubsystem;

  public LowIntakeConePreparation(ElevatorSubsystem elevatorSubsystem) {
   

   this.elevatorSubsystem = elevatorSubsystem;
   
    // Use addRequirements() here to declare subsystem dependencies.


    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turretSubsystem.setPosition(0);
    // boomSubsystem.setPosition(Constants.BoomConstants.kBoomIntakeReach);
    elevatorSubsystem.setPosition(Constants.ElevatorConstants.kConeLowIntake);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
