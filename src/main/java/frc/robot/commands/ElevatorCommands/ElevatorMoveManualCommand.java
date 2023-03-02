// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This command it used for testing and finding set points, it will likely stay here, but will not be used
package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;
import frc.robot.HelperClasses.*;

public class ElevatorMoveManualCommand extends CommandBase {
  /** Creates a new ElevatorMoveManualCommand. */
  ElevatorSubsystem elevatorSubsystem;
  Supplier<Double> joystickY;

  SignalProcessor signalProcessor = new SignalProcessor(10, 0.05, 0);

  public ElevatorMoveManualCommand(ElevatorSubsystem subsystem, Supplier<Double> joystickY) {

    elevatorSubsystem = subsystem;
    this.joystickY = joystickY;

    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = signalProcessor.getOutput(joystickY.get());
    elevatorSubsystem.setVoltage(output);
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
