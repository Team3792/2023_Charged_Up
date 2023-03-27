// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTaxi extends CommandBase {
  /** Creates a new DriveTaxi. */
  DriveSubsystem driveSubsystem;
  public DriveTaxi(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.zeroSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setVoltage(-4, -4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopAndBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
    driveSubsystem.getTotalDistance() > Constants.DriveConstants.kAutoTaxiDistanceMeters;
  }
}
