// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.time.LocalTime;
import java.time.temporal.ChronoUnit;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HelperClasses.RobotLayout;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonCamera leftCamera;
  private PhotonCamera rightCamera;
  private PhotonPoseEstimator leftPhotonPoseEstimator;
  private PhotonPoseEstimator rightPhotonPoseEstimator;
  public  Pose2d turretPose = new Pose2d();
  public boolean seesTargets = false;
  public boolean twoCameraMode = true;
  public double visionReliabilityIndex = 0.0;
  public LocalTime lastVisionUpdate = LocalTime.now();
 // private final Field2d field = new Field2d();

  public VisionSubsystem() {
    // The parameter for loadFromResource() will be different depending on the game.
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch(Exception e) {

    }
    // If two camera mode is enabled, we need to create two PhotonCamera objects and two PhotonPoseEstimator objects. Else, only use left camera.
    if (twoCameraMode) {

      // Left Camera
      leftCamera = new PhotonCamera("leftCam");
      Transform3d robotToCamLeft = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, leftCamera, robotToCamLeft);

      // Right Camera 
      rightCamera = new PhotonCamera("rightCam");
      Transform3d robotToCamRight = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, rightCamera, robotToCamRight);
    } else {
      // Left Camera
      leftCamera = new PhotonCamera("leftCam");
      Transform3d robotToCamLeft = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, leftCamera, robotToCamLeft);
    }
    // SmartDashboard.putData("Field", field);
  }

  private void getRobotPose(){

    // if twoCameraMode is enabled, we need to update both cameras. Else, only update left camera. seesTarget will reflect this as well.
    if (twoCameraMode) {
      //Updating the seeTargets bool to the cameras result
      seesTargets = leftCamera.getLatestResult().hasTargets() || rightCamera.getLatestResult().hasTargets();
      if (seesTargets) {
        lastVisionUpdate = LocalTime.now();
      }

      leftPhotonPoseEstimator.setLastPose(turretPose);
      rightPhotonPoseEstimator.setLastPose(turretPose);

      Optional<EstimatedRobotPose> currentPoseLeft = leftPhotonPoseEstimator.update();
      Optional<EstimatedRobotPose> currentPoseRight = rightPhotonPoseEstimator.update();

      //  Four Scenarios: 1) Both cameras see targets, 2) Only left sees targets, 3) Only right sees targets, 4) Neither sees targets
      // 1) Both cameras see targets. In that case, we want to use the average of the two poses.
      if (currentPoseLeft.isPresent() && currentPoseRight.isPresent()) {
        turretPose = getAveragePose(currentPoseLeft.get().estimatedPose.toPose2d(), currentPoseRight.get().estimatedPose.toPose2d());
      }
      // 2) Only left sees targets. In that case, we want to use the left pose.
      else if (currentPoseLeft.isPresent()) {
        turretPose = currentPoseLeft.get().estimatedPose.toPose2d();
      }
      // 3) Only right sees targets. In that case, we want to use the right pose.
      else if (currentPoseRight.isPresent()) {
        turretPose = currentPoseRight.get().estimatedPose.toPose2d();
      }
      // 4) Neither sees targets. In that case, we want to use the last pose.
      else {
        //  I know this is redundant, but I'm leaving it in for clarity.
        turretPose = turretPose;
      }

    } else {
      //Updating the seeTargets bool to the cameras result
      seesTargets = leftCamera.getLatestResult().hasTargets();

      if (seesTargets) {
        lastVisionUpdate = LocalTime.now();
      }

      leftPhotonPoseEstimator.setLastPose(turretPose);

      Optional<EstimatedRobotPose> currentPoseLeft = leftPhotonPoseEstimator.update();

      //  Two Scenarios: 1) Left sees targets, 2) Left doesn't see targets
      // 1) Left sees targets. In that case, we want to use the left pose.
      if (currentPoseLeft.isPresent()) {
        turretPose = currentPoseLeft.get().estimatedPose.toPose2d();
      }
      // 2) Left doesn't see targets. In that case, we want to use the last pose.
      else {
        //  I know this is redundant, but I'm leaving it in for clarity.
        turretPose = turretPose;
      }
    }

  }

  private Pose2d getAveragePose(Pose2d leftPose, Pose2d rightPose) {
    Pose2d deviation = rightPose.relativeTo(leftPose);
    Translation2d translationDeviation = deviation.getTranslation();
    Rotation2d rotationDeviation = deviation.getRotation();
    Translation2d averageTranslation = new Translation2d(translationDeviation.getX() / 2, translationDeviation.getY() / 2);
    Rotation2d averageRotation = new Rotation2d(rotationDeviation.getRadians() / 2);
    Transform2d translation = new Transform2d(averageTranslation, averageRotation);
    return leftPose.transformBy(translation);
  }

  // This method can be used to find the reliability of the vision system. It will return a value between 0 and 1, with 1 being the most reliable. The more recent the result, the closer the value to 1. The older the result, the closer the value to 0. Anything beyond 3 seconds is 0. 
  private double getVisionReliabilityIndex() {
    LocalTime currentTime = LocalTime.now();
    long secondsSinceLastUpdate = ChronoUnit.SECONDS.between(lastVisionUpdate, currentTime);
    if (secondsSinceLastUpdate > 3) {
      return 0.0;
    } else {
      return 1.0 - (secondsSinceLastUpdate / 3.0);
    }
    
  }

  public void updateAndGetTurretPose() {
    // This method will be called once per scheduler run
    getRobotPose();
    SmartDashboard.putNumber("VisionReliability", getVisionReliabilityIndex());
  }
}
