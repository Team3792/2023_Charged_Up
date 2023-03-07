//PURPOSE OF COMMAND:
//This command is intended to be a quicker version of the turret turn, intended for high speed purposes
//this is theoretical, this might not be mechanically possible


//TODO:
//      push the robot to the limit to see what turn speed we can handle
//      "move fast and break things" --Zuckerberg probably

package frc.robot.commands.UnusedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class QuickTurretTurnCommand extends PIDSubsystem {
  /** Creates a new QuickTurretTurnCommand. */
  public QuickTurretTurnCommand() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
