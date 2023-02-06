//PURPOSE OF COMMAND:
//this command is intended to run the portion of the intake that can grab a cone

//TODO:
//      decide motor speed - Done (see constants)



package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeConeCommand extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  
  /** Creates a new IntakeConeCommand. */
  public  IntakeConeCommand(IntakeSubsystem subsystem) {

    intakeSubsystem = subsystem;
  
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    RobotContainer.intakeStatus = "intaking cone";

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.coneIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeStatus = "cone";
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
