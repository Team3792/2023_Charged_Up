// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakePreparationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AdjustForConeIntakeCommand extends CommandBase {
  /** Creates a new AdjustForCubeIntakeCommand. */
  ElevatorSubsystem elevatorSubsystem;

  public AdjustForConeIntakeCommand(ElevatorSubsystem subsystem) {

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
      elevatorSubsystem.setSetPoint(Constants.ElevatorConstants.kConeLowIntake);
    }else if(RobotContainer.lastIntakeHeight == "high"){

      elevatorSubsystem.setSetPoint(Constants.ElevatorConstants.kConeHighIntake);

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
