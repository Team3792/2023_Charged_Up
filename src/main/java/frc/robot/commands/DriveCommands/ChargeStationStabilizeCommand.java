// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class ChargeStationStabilizeCommand extends CommandBase {
  /** Creates a new ChargeStationStabilizeCommand. */
  DriveSubsystem driveSubsystem;

  PIDController pidController = new PIDController(Constants.ChargeStationStabilizeConstants.kPIDkP, 0, 0);

  public ChargeStationStabilizeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get pitch from driveSubsystem's pigeon
    double pitch = driveSubsystem.getPitch();

    //Calculate the next term fromt the PID
    double forwardVelocity = pidController.calculate(pitch);

    //Set the driveSubsystem to drive with that scalar
    driveSubsystem.differentialDrive.arcadeDrive(forwardVelocity, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopAndBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
