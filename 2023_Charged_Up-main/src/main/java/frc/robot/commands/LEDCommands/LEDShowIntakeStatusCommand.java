// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;

public class LEDShowIntakeStatusCommand extends CommandBase {
  /** Creates a new LEDShowIntakeStatusCommand. */
  //FIGURE OUT HOW TO NAME THIS, MAYBE led instead of LED?
  private LEDSubsystem ledSubsystem;
  public LEDShowIntakeStatusCommand(LEDSubsystem subsystem) {

    ledSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String intakeStatus = RobotContainer.intakeStatus;
//figure out the right codes later, work with Julie - maybe put these in constants?

    switch(intakeStatus){
      case "cube":
      ledSubsystem.sendCode(0);
        break;
      case "cone":
      ledSubsystem.sendCode(0);
        break;
      case "intake cube":
      ledSubsystem.sendCode(0);
        break;
      case "intaking cone":
      ledSubsystem.sendCode(0);
        break;
      case "dropping":
      ledSubsystem.sendCode(0);
        break;
      case "none":
      ledSubsystem.sendCode(0);
        break;
    }


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
