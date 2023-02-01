//PURPOSE OF COMMAND:
//This is the default drive command. Basic PID driving happens here
//Takes Driver Joystick's two axis as double suppliers and outputs the driving paramaters.

//TODO:
//       PID tuning
//       test on actual bot (need to wait for chassis development)


package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  Supplier<Double> joystickForward, joystickRotation;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, Supplier<Double> joystickForward, Supplier<Double> joystickRotation) {
    this.joystickForward = joystickForward;
    this.joystickRotation = joystickRotation;
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(joystickForward.get(), joystickRotation.get());

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
