//PURPOSE OF COMMAND:
//This is intended to turn the turret back to the initial front facing position 
//it will need to continually keep track of encoders of the belt turret in order to know where it needs to move


//TODO:
//      figure out the running tracker for turret angle
//      figure out speed of turning back
//      what's a good button for the this?

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetTurretCommand extends CommandBase {
  /** Creates a new ResetTurretCommand. */
  public ResetTurretCommand() {
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
