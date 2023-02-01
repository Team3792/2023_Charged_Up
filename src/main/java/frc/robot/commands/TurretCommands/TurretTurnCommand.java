//PURPOSE OF COMMAND:
//This command is intended to turn the turret at a regular speed, intended to allow precision placement
//this will use keep LINEAR SPEED THE SAME! as the boom extends, angular speed MUST change


//TODO:
//      figure how to link arm extension with slowing angular speed
//      find good linear speed that we want


package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretTurnCommand extends CommandBase {
  /** Creates a new TurretTurnCommand. */
  public TurretTurnCommand() {
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
