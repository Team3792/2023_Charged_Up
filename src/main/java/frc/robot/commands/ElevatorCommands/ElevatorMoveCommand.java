//PURPOSE OF COMMAND:
//This is a command to move the elevator on it's up and down axis
//This will most likely have two modes:
//One: manual control. This will involve an axis on the Operator controller to move it up and down
//Two: Auto setpoints of Low, Mid, and High that bring it to each of the 3 scoring levels. 

//TODO:
//        Decide whether PID or not
//        Get encoder clicks to meters to know conversion
//        decide whether this will be one or two commands 

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorMoveCommand extends CommandBase {
  /** Creates a new ElevatorMoveCommand. */
  public ElevatorMoveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
