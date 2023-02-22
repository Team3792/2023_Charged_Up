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
import java.util.function.Supplier;


public class IntakeConeCommand extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  Supplier<Double> intakeCurrent;

  
  /** Creates a new IntakeConeCommand. */
  public  IntakeConeCommand(IntakeSubsystem subsystem, Supplier<Double> intakeCurrent) {

    intakeSubsystem = subsystem;
    this.intakeCurrent = intakeCurrent;
  
    
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
    intakeSubsystem.coneIntake();
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
    //If the intakeCurrent is too high, return true for isFinished, ending the command
    return (intakeCurrent.get() > Constants.PowerDistributionHubConstants.kConeShutoffCurrent);
  }
}
