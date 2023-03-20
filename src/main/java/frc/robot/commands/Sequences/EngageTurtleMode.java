// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorToTurtleMode;
import frc.robot.commands.TurretCommands.TurretToTurtleMode;
import frc.robot.subsystems.BoomSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.BoomCommands.BoomToTurtleMode;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EngageTurtleMode extends SequentialCommandGroup {
  /** Creates a new EngageTurtleMode. */
  TurretSubsystem turretSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  BoomSubsystem boomSubsystem;

  public EngageTurtleMode(TurretSubsystem turretSubsystem, ElevatorSubsystem elevatorSubsystem, BoomSubsystem boomSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.boomSubsystem = boomSubsystem;

 
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorToTurtleMode(elevatorSubsystem),
      new TurretToTurtleMode(turretSubsystem)
      
    );
  }
}