// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines.UnusedRoutines;

import org.ejml.dense.row.decomposition.BaseDecomposition_FDRB_to_FDRM;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.Actions.DropAllAutoCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.BoomCommands.BoomFromTurtleMode;
import frc.robot.commands.Sequences.EngageTurtleMode;
import frc.robot.commands.Sequences.ToElevatorLevel;
import frc.robot.subsystems.BoomSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Autonomous.Actions.DriveTaxi;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeTaxi extends SequentialCommandGroup {
  /** Creates a new ConeTaxi. */
  DriveSubsystem driveSubsystem;
  TurretSubsystem turretSubsystem;
  BoomSubsystem boomSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;

  public ConeTaxi(DriveSubsystem driveSubsystem, TurretSubsystem turretSubsystem, BoomSubsystem boomSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
   
   this.driveSubsystem = driveSubsystem;
   this.turretSubsystem = turretSubsystem;
   this.boomSubsystem = boomSubsystem;
   this.intakeSubsystem = intakeSubsystem;
   this.elevatorSubsystem = elevatorSubsystem;

   addRequirements(driveSubsystem, turretSubsystem, boomSubsystem, intakeSubsystem, elevatorSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToElevatorLevel(elevatorSubsystem, turretSubsystem, boomSubsystem, 1, "cone"),
      new DropAllAutoCommand(intakeSubsystem, "cone"),
      new EngageTurtleMode(turretSubsystem, elevatorSubsystem, boomSubsystem),
      new DriveTaxi(driveSubsystem)
    );
  }
}
