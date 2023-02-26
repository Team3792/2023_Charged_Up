//PURPOSE OF COMMAND:
//this command is intended to run the portion of the intake that can grab a cube

//TODO:
//      decide motor speed - Done (see constants)

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

public class IntakeCubeCommand extends CommandBase {
  /** Creates a new IntakeCubeCommand. */
  IntakeSubsystem intakeSubsystem;
  Supplier<Double> intakeCurrent;

  

  public IntakeCubeCommand(IntakeSubsystem subsystem, Supplier<Double> intakeCurrent) {
    intakeSubsystem = subsystem;
    this.intakeCurrent = intakeCurrent;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);

   // SmartDashboard.putRaw(getName(), null)

    //Returning object being intaked so that this value can be used elsewhere

    
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.slewRateLimiter.reset(0);
    RobotContainer.intakeStatus = "intaking cube";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeSubsystem.cubeIntake();
    //  System.out.println(intakeCurrent.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();

    RobotContainer.intakeStatus = "cone";
   SmartDashboard.putString("Intake", "Cone");
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If the intakeCurrent is too high, return true for isFinished, ending the command
    return (intakeCurrent.get() > Constants.PowerDistributionHubConstants.kCubeShutoffCurrent);
  }
}
