//PURPOSE OF COMMAND:
//this command is intended to run the portion of the intake that can grab a cube

//TODO:
//      decide motor speed - Done (see constants)

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class IntakeCubeCommand extends CommandBase {
  /** Creates a new IntakeCubeCommand. */
  IntakeSubsystem intakeSubsystem;
  PowerDistribution powerDistribution = new PowerDistribution(Constants.MotorID.kPowerDistribution, ModuleType.kRev);

  

  public IntakeCubeCommand(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);

   // SmartDashboard.putRaw(getName(), null)

    //Returning object being intaked so that this value can be used elsewhere

    
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(powerDistribution.getCurrent(Constants.PowerDistributionHubConstants.kPDHIntakeChannel) < Constants.PowerDistributionHubConstants.kCubeShutoffCurrent){
      intakeSubsystem.cubeIntake();
      }else {
        //If the current draw is too much, end the command and break the intake
        end(true);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    SmartDashboard.putString("Intake", "Cube");
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
