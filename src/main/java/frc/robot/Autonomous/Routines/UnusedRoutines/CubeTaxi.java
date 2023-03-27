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
import frc.robot.commands.TurretCommands.TurretOutOfTurtleMode;
import frc.robot.subsystems.BoomSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Autonomous.Actions.ChargeStationStabilizeCommand;
import frc.robot.Autonomous.Actions.DrivePath;
import frc.robot.Autonomous.Actions.DriveTaxi;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import java.nio.file.FileSystem;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeTaxi extends SequentialCommandGroup {
  /** Creates a new ConeTaxi. */
  DriveSubsystem driveSubsystem;
  TurretSubsystem turretSubsystem;
  BoomSubsystem boomSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  DrivePath drivePath;

  public CubeTaxi(DriveSubsystem driveSubsystem, TurretSubsystem turretSubsystem, BoomSubsystem boomSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
   
   this.driveSubsystem = driveSubsystem;
   this.turretSubsystem = turretSubsystem;
   this.boomSubsystem = boomSubsystem;
   this.intakeSubsystem = intakeSubsystem;
   this.elevatorSubsystem = elevatorSubsystem;

   drivePath = new DrivePath(driveSubsystem);

   Trajectory overChargeStationAndBack = new Trajectory();
   try {
    Path  overChargeStationAndBackPath= Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/OverChargeStationAndBack.wpilib.json");
    overChargeStationAndBack = TrajectoryUtil.fromPathweaverJson(overChargeStationAndBackPath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + "pathplanner/generatedJSON/ToCone2.wpilib.json", ex.getStackTrace());
  }

   addRequirements(driveSubsystem, turretSubsystem, boomSubsystem, intakeSubsystem, elevatorSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     // new ToElevatorLevel(elevatorSubsystem, turretSubsystem, boomSubsystem, 1, "cube"),
     new TurretOutOfTurtleMode(turretSubsystem),
      new DropAllAutoCommand(intakeSubsystem, "cone"),
     // new EngageTurtleMode(turretSubsystem, elevatorSubsystem, boomSubsystem),
     drivePath.runPath(overChargeStationAndBack),
    new ChargeStationStabilizeCommand(driveSubsystem)
    );
  }
}
