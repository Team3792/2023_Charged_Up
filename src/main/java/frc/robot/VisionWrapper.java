// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionWrapper {
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonCamera leftCam;//, rightCam;
  private PhotonPoseEstimator photonPoseEstimatorLeft, photonPoseEstimatorRight;
  public Pose2d robotPose = new Pose2d();
  PhotonTrackedTarget target;
  public Transform3d pose = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), new Pose3d(1, 0, 0, new Rotation3d(0, 0, 0)));
  

 
  private final Field2d m_field = new Field2d();
  public VisionWrapper() {
    // The parameter for loadFromResource() will be different depending on the game.
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch(Exception e) {

    }
    // Left Camera
    leftCam = new PhotonCamera("leftCam");
   // rightCam = new PhotonCamera("rightCam");
    Transform3d robotToCamLeft = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
    //Transform3d robotToCamRight = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0, Math.PI/2));
     //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, leftCam, robotToCamLeft);
    //photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, rightCam, robotToCamRight);

    SmartDashboard.putData("Field", m_field);
  }
  
  public void periodic() {

    var resultLeft = leftCam.getLatestResult();
   // var resultRight = rightCam.getLatestResult();

    if(resultLeft.hasTargets()){
    target = resultLeft.getBestTarget();
     pose = target.getBestCameraToTarget();
    }//else if(resultRight.hasTargets()){
    //   target = resultRight.getBestTarget();
    //   pose = target.getBestCameraToTarget();
     
    // }

    
  


  //   This method will be called once per scheduler run
  //   photonPoseEstimator.setLastPose(robotPose);
  //   Optional<EstimatedRobotPose> currentPose = photonPoseEstimator.update();
  //   if (currentPose.isPresent()) {
  //     Pose2d camPose = currentPose.get().estimatedPose.toPose2d();
  //    System.out.println(camPose);
  //     robotPose = currentPose.get().estimatedPose.toPose2d();
  //     m_field.setRobotPose(robotPose);
  //   }

  //   Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
  // distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));
  //   m_field.setRobotPose(currentPose.);
  }
}
