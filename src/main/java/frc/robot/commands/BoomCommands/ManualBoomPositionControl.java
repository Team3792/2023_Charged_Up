// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BoomCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BoomSubsystem;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.HelperClasses.*;

public class ManualBoomPositionControl extends CommandBase {
  /** Creates a new ManualBoomPositionControl. */
  BoomSubsystem boomSubsystem;
  Supplier<Double> joystickInput;
  SignalProcessor signalProcessor = new SignalProcessor(Constants.BoomConstants.kBoomMaxReach, 0, 0);
  public ManualBoomPositionControl(BoomSubsystem boomSubsystem, Supplier<Double> joystickInput) {
    this.boomSubsystem = boomSubsystem;
    this.joystickInput = joystickInput;

    addRequirements(this.boomSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = signalProcessor.getOutput(joystickInput.get());
    boomSubsystem.setPosition(output);
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
