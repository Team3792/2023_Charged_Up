//PURPOSE OF COMMAND:
//this command is intended to run the portion of the intake that can grab a cone

//TODO:
//      decide motor speed - Done (see constants)



package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeConeCommand extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  PowerDistribution powerDistribution = new PowerDistribution(Constants.MotorID.kPowerDistribution, ModuleType.kRev);
  
  /** Creates a new IntakeConeCommand. */
  public  IntakeConeCommand(IntakeSubsystem subsystem) {

    intakeSubsystem = subsystem;
  
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(powerDistribution.getCurrent(Constants.PowerDistributionHubConstants.kPDHIntakeChannel) < Constants.PowerDistributionHubConstants.kConeShutoffCurrent){
    intakeSubsystem.coneIntake();
    }else {
      //If the current draw is too much, end the command and break the intake
      end(true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Intake", "Cube");
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
