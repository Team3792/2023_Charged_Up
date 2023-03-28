// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorMoveAutoCommand;
import frc.robot.commands.TurretCommands.TurretOutOfTurtleMode;
import frc.robot.subsystems.BoomSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToElevatorLevel extends SequentialCommandGroup {
  /** Creates a new ToElevatorLevel. */
  ElevatorSubsystem elevatorSubsystem;
  TurretSubsystem turretSubsystem;
  BoomSubsystem boomSubsystem;




  public ToElevatorLevel(ElevatorSubsystem elevatorSubsystem, TurretSubsystem turretSubsystem, BoomSubsystem boomSubsystem, int elevatorIntendedHeight, String intakeStatus) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.elevatorSubsystem = elevatorSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.boomSubsystem = boomSubsystem;

    addCommands(
      new TurretOutOfTurtleMode(turretSubsystem),
      new ElevatorMoveAutoCommand(elevatorSubsystem, intakeStatus, elevatorIntendedHeight)
    );
  }
}
