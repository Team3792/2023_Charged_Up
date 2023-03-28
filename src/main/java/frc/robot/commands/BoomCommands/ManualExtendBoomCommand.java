//PURPOSE OF COMMAND:
//This command is intended to move the arm (sometimes referred to as the "boom") up and down it's axis
//It will take a double supplier from the Operator Joystick (TBD which axis) and then move the arm up and down

//TODO:
//       Decide whether PID or not (prob yes)
//       Decide joystick axis
//       Create instant command to bind button in robot container

package frc.robot.commands.BoomCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import javax.swing.text.StyledEditorKit.BoldAction;

import frc.robot.Constants;
import frc.robot.HelperClasses.*;

import frc.robot.subsystems.BoomSubsystem;
public class ManualExtendBoomCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */
  private BoomSubsystem boomSubsystem;
  private Supplier<Double> joystickXRotation;
  SignalProcessor signalProcessor = new SignalProcessor(Constants.BoomConstants.kMaxCruiseVoltage, 0.05, 1);



  public ManualExtendBoomCommand(BoomSubsystem subsystem, Supplier<Double> joystickXRotation) {

    boomSubsystem = subsystem;
    this.joystickXRotation = joystickXRotation;


    addRequirements(boomSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//if(engage.get()){
    double rawInput = joystickXRotation.get();
    double processedInput = signalProcessor.getOutput(rawInput);
    boomSubsystem.setVoltage(processedInput);
//}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
