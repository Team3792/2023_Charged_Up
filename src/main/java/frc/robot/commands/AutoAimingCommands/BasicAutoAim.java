// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionWrapper;
import frc.robot.subsystems.TurretSubsystem;
import java.lang.Math;

import org.photonvision.targeting.PhotonTrackedTarget;

public class BasicAutoAim extends CommandBase {
  /** Creates a new BasicAutoAim. */
  TurretSubsystem turretSubsystem;
  VisionWrapper vision = new VisionWrapper();
  Transform3d pose;


  public BasicAutoAim(TurretSubsystem subsystem) {
    turretSubsystem = subsystem;
    addRequirements(turretSubsystem);
   // addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.periodic();

     pose = vision.pose;
     double angle = Math.atan(pose.getY()/pose.getX()) * 180/Math.PI;

     // + turretSubsystem.getAngleDegrees();


   System.out.println(angle);
  turretSubsystem.setPosition(angle + turretSubsystem.getAngleDegrees());

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
