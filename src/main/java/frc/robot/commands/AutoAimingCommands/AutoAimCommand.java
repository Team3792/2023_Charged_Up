// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimingCommands;


import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.HelperClasses.*;
import frc.robot.subsystems.BoomSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class AutoAimCommand extends CommandBase {
  /** Creates a new AutoAim. */
  ConeTargetFinder coneTargetFinder = new ConeTargetFinder();
  CubeTargetFinder cubeTargetFinder = new CubeTargetFinder();
  TurretSubsystem turretSubsystem;
  BoomSubsystem boomSubsystem;
  VisionSubsystem visionSubsystem;
  Pose2d turretPose;
  
  Pose2d target;
  

  public AutoAimCommand(TurretSubsystem turretSubsystem, BoomSubsystem boomSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.boomSubsystem = boomSubsystem;
    


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
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    turretPose = new Pose2d(0, 0, new Rotation2d(0));
    if(RobotContainer.lastPieceHeld == "cube"){
     target = cubeTargetFinder.getTarget(RobotContainer.elevatorHeight, turretPose.getY());
    }else if(RobotContainer.lastPieceHeld == "cone"){
     target = coneTargetFinder.getTarget(RobotContainer.elevatorHeight, turretPose.getY());
    }
    Pose2d targetPose2d = new Pose2d(target.getX(), target.getY(), new Rotation2d(0));
    //visionSubsystem.addTarget(targetPose2d);

    aimAtTarget(target);


  }

  private void aimAtTarget(Pose2d target){
    //Finding angle of target -> turret center ->Red alliance wall (ccw rotation)
    double targetToTurretCenterSlope = (target.getY() - turretPose.getY())/(target.getX() - turretPose.getX());
    double targetToTurretCenterAngle = Math.atan(targetToTurretCenterSlope);
    //Correct slope for the other possible angle atan could be
    if(targetToTurretCenterSlope < 0){
      targetToTurretCenterAngle = Math.PI + targetToTurretCenterAngle;
    }

    //Finding the angle the turret should turn (converting targetToTurretCenterAngle to turret relative)
    double turretRelativeTargetAngle = targetToTurretCenterAngle - turretPose.getRotation().getRadians();

    double turretRelativeTargetAngleDegrees = Math.toDegrees(turretRelativeTargetAngle);

    //setting turret to that angle
    //Note: add 
    turretSubsystem.setSetPointDegrees(turretRelativeTargetAngleDegrees);

    //Finding absolute distance from turret center to target

    double distanceTurretToTarget = Math.hypot(
      turretPose.getX() - target.getX(),
      turretPose.getY() - target.getY() 
      );


    //sending this to boom to set this distance

    boomSubsystem.setPositionDistance(distanceTurretToTarget);


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
