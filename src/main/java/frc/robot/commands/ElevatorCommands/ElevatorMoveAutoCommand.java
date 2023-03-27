//PURPOSE OF COMMAND:
//This is a command to move the elevator on it's up and down axis
//This will most likely have two modes:
//One: manual control. This will involve an axis on the Operator controller to move it up and down
//Two: Auto setpoints of Low, Mid, and High that bring it to each of the 3 scoring levels. 

//TODO:
//        Decide whether PID or not - Done
//        Get encoder clicks to meters to know conversion - Don't think this is needed (we can measure things in encoder ticks)
//        decide whether this will be one or two commands - 2 Commands, done

package frc.robot.commands.ElevatorCommands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorMoveAutoCommand extends CommandBase {
  /** Creates a new ElevatorMoveCommand. */
private ElevatorSubsystem elevatorSubsystem;
int situationKey;
/*
0, 1, 2 are cube ground, mid, high, repectively

3, 4, 5 are cone ground, mid, high, respectively
 */
  public ElevatorMoveAutoCommand(ElevatorSubsystem subsystem, String intakeStatus, int height) {
    elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
    //Default to cone height
    situationKey = (intakeStatus == "cube")? 0:3 + height;
   

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int goalHeightEncoder = 0;

    switch(situationKey){
      case 0:
        goalHeightEncoder = Constants.ElevatorConstants.kCubeGroundHeight;
        break;
      case 1:
        goalHeightEncoder = Constants.ElevatorConstants.kCubeMiddleHeight;
        break;
      case 2:
        goalHeightEncoder = Constants.ElevatorConstants.kCubeHighHeight;
        break;
      case 3:
        goalHeightEncoder = Constants.ElevatorConstants.kConeGroundHeight;
        break;
      case 4:
        goalHeightEncoder = Constants.ElevatorConstants.kConeMiddleHeight;
        break;
      case 5:
        goalHeightEncoder = Constants.ElevatorConstants.kConeHighHeight;
        break;

    }
System.out.println(goalHeightEncoder);

    elevatorSubsystem.setSetPoint(Constants.ElevatorConstants.kCubeMiddleHeight);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.arrived;
  }
}
