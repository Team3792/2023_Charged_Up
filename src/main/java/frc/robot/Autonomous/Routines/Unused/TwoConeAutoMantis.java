// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines.Unused;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Autonomous.Actions.DrivePath;
import frc.robot.Autonomous.Actions.DropAllAutoCommand;
import frc.robot.commands.IntakeCommands.IntakeConeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import java.io.IOException;
import java.nio.file.Path;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeAutoMantis extends SequentialCommandGroup {
  /** Creates a new OneConeAuto. */
  DriveSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  Trajectory toCone, fromCone;
  DrivePath drivePath;
  PowerDistribution powerDistribution;
  public TwoConeAutoMantis(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PowerDistribution powerDistribution) {
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.powerDistribution = powerDistribution;
    drivePath = new DrivePath(driveSubsystem);

    loadTrajectories();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Drop cone off, go to cone, pick it up, drive back, drop off again

    addCommands(
    new DropAllAutoCommand(intakeSubsystem, "cone"),
      drivePath.runPath(toCone),
    new IntakeConeCommand(intakeSubsystem, () -> powerDistribution.getCurrent(Constants.PowerDistributionHubConstants.kPDHIntakeChannel)),
    drivePath.runPath(fromCone),
    new DropAllAutoCommand(intakeSubsystem, "cone")
    );
  }

  private void loadTrajectories(){
//load trajectories, and report errors
  try {
    Path toConePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/ToCone2.wpilib.json");
    toCone = TrajectoryUtil.fromPathweaverJson(toConePath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + "pathplanner/generatedJSON/ToCone2.wpilib.json", ex.getStackTrace());
  }

  try {
    Path fromConePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/FromCone2.wpilib.json");
    fromCone = TrajectoryUtil.fromPathweaverJson(fromConePath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + "pathplanner/generatedJSON/FromCone2.wpilib.json", ex.getStackTrace());
  }

  }
}
