// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimingCommands;

import frc.robot.HelperClasses.*;
import frc.robot.subsystems.BoomSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  ConeTargetFinder coneTargetFinder = new ConeTargetFinder();
  CubeTargetFinder cubeTargetFinder = new CubeTargetFinder();
  TurretSubsystem turretSubsystem;
  BoomSubsystem boomSubsystem;
  VisionSubsystem visionSubsystem;

  public AutoAim(TurretSubsystem turretSubsystem, BoomSubsystem boomSubsystem, VisionSubsystem visionSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.boomSubsystem = boomSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(boomSubsystem);
    addRequirements(turretSubsystem);
    addRequirements(visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //Elevator heights standard:
    //0, ground
    //1, mid
    //2, high

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //public  Pose2d robotPose = new Pose2d();
    Pose2D robotPose = visionSubsystem.robotPose;
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
