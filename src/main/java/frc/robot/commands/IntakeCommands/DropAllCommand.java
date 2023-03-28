// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DropAllCommand extends CommandBase {
  /** Creates a new DropAllCommand. */
  IntakeSubsystem intakeSubsystem;


  public DropAllCommand(IntakeSubsystem subsystem, String intakeStatus) {

    intakeSubsystem = subsystem;
 


    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeStatus = "dropping";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Checking intake status and extaking accordingly

    if(RobotContainer.intakeStatus == "cube"){
      intakeSubsystem.cubeExtake();
    }else if(RobotContainer.intakeStatus == "cone"){
      intakeSubsystem.coneExtake();
    }
    
    // if(SmartDashboard.getString("Intake", "none")  == "Cube"){
    //   intakeSubsystem.cubeExtake();
    // }else if(SmartDashboard.getString("Intake", "none") == "Cone"){
    //   intakeSubsystem.coneExtake();
    // }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //After dropping, set intake status to "none"
    RobotContainer.intakeStatus = "none";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
