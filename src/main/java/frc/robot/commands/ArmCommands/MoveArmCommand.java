//PURPOSE OF COMMAND:
//This command is intended to move the arm (sometimes referred to as the "boom") up and down it's axis
//It will take a double supplier from the Operator Joystick (TBD which axis) and then move the arm up and down

//TODO:
//       Decide whether PID or not (prob yes)
//       Decide joystick axis
//       Create instant command to bind button in robot container

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */
  public MoveArmCommand() {
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
