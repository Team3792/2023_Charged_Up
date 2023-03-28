// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakePreparationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AdjustForCubeIntakeCommand extends CommandBase {
  /** Creates a new AdjustForCubeIntakeCommand. */
  ElevatorSubsystem elevatorSubsystem;

  public AdjustForCubeIntakeCommand(ElevatorSubsystem subsystem) {

    elevatorSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.lastIntakeHeight == "low"){
      elevatorSubsystem.setPosition(Constants.ElevatorConstants.kCubeLowIntake);
    }else if(RobotContainer.lastIntakeHeight == "high"){

      elevatorSubsystem.setPosition(Constants.ElevatorConstants.kCubeHighIntake);

    }
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
