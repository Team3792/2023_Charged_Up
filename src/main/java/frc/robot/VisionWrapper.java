// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionWrapper {
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonCamera leftCam;
  private PhotonPoseEstimator photonPoseEstimator;
  private Pose2d robotPose = new Pose2d();
  private final Field2d m_field = new Field2d();

  public VisionWrapper() {
    // The parameter for loadFromResource() will be different depending on the game.
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch(Exception e) {

    }
    // Left Camera
    leftCam = new PhotonCamera("leftCam");
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, leftCam, robotToCam);
    SmartDashboard.putData("Field", m_field);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    photonPoseEstimator.setLastPose(robotPose);
    Optional<EstimatedRobotPose> currentPose = photonPoseEstimator.update();
    if (currentPose.isPresent()) {
      Pose2d camPose = currentPose.get().estimatedPose.toPose2d();
      System.out.println(camPose);
      robotPose = currentPose.get().estimatedPose.toPose2d();
      m_field.setRobotPose(robotPose);
    }
    // m_field.setRobotPose(currentPose.);
  }
}
