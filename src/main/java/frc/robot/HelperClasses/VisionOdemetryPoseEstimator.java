// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.HelperClasses;
import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class VisionOdemetryPoseEstimator {

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    //TurretSubsystem turretSubsystem;
    public Pose2d turretLocation = new Pose2d();
    public Pose2d chassisLocation = new Pose2d();


    public VisionOdemetryPoseEstimator(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem){

        //We don't send anything to the drive or vision subsystme, so we don't use addRequirements as that would stop the commands commanding these subsystems

        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        //this.turretSubsystem = turretSubsystem;

    }

    public void update(){

    //If vision is solid, update the odometry, otherwise, use the odemtry to find turret position
    visionSubsystem.updateAndGetTurretPose();   
    if(visionSubsystem.seesTargets){
                  System.out.println("Sees targets, updating od"); 
            turretLocation = visionSubsystem.turretPose;
           // findChassisLocationFromVision();
           chassisLocation = turretLocation;
            updateOdometryFromVisionLocation();
        }else{
            findTurretLocationFromOdemetry();
        }
    }

    private void findTurretLocationFromOdemetry(){
        // Translation2d chassisToTurretGeneral = new Translation2d(-.3, -0);

        // double chassisToFieldAngleDegrees = chassisLocation.getRotation().getDegrees();
        // double chassisToTurretAngleDegrees = turretSubsystem.getAngleDegrees();
        // double fieldRelativeTurretRotation = chassisToFieldAngleDegrees + chassisToTurretAngleDegrees;


        // Transform2d chassisToTurretCurrent = 
        // new Transform2d(chassisToTurretGeneral.rotateBy(
        //     new Rotation2d(Math.toDegrees(chassisToFieldAngleDegrees))), 
        //     new Rotation2d(chassisToTurretAngleDegrees)
        //     );

        // chassisLocation = turretLocation.plus(chassisToTurretCurrent);
        chassisLocation = driveSubsystem.robotPose;
     
    }

    private void findChassisLocationFromVision(){
      //turret is some constant ahead of chassis center
      //TODO: replace 0.3 with accuarate measurement (From CAD)
    //   Translation2d turretToChassisGeneral = new Translation2d(.3, -0);

    //   double turretToFieldAngleDegrees = turretLocation.getRotation().getDegrees();
    //   double chassisToTurretAngleDegrees = turretSubsystem.getAngleDegrees();
    //   double fieldRelativeChassisRotation = turretToFieldAngleDegrees - chassisToTurretAngleDegrees;


    //   Transform2d turretToChassisCurrent = 
    //   new Transform2d(turretToChassisGeneral.rotateBy(
    //       new Rotation2d(Math.toDegrees(fieldRelativeChassisRotation))), 
    //       new Rotation2d(-chassisToTurretAngleDegrees)
    //       );

    //   chassisLocation = turretLocation.plus(turretToChassisCurrent);
   
    
    }

    private void updateOdometryFromVisionLocation(){
        //Resetting the gryo (pigeon) and the odemetry with new locations
        double newAngle = chassisLocation.getRotation().getDegrees();

        driveSubsystem.pigeon.reset();


        driveSubsystem.zeroSensors();
        driveSubsystem.differentialDriveOdometry.resetPosition(
            new Rotation2d(0), 
            0, 
            0, 
            chassisLocation);
    }






}
