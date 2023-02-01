//PURPOSE OF COMMAND:
//this command is intended to run the portion of the intake that can grab a cube

//TODO:
//      decide motor speed

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCubeCommand extends CommandBase {
  /** Creates a new IntakeCubeCommand. */
  public IntakeCubeCommand() {
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
