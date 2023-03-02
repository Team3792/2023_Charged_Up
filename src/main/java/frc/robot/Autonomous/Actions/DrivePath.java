// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Actions;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


/** Add your docs here. */
public class DrivePath {
    //Pass in the drive subsystem
    DriveSubsystem driveSubsystem;

    public DrivePath(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }

    public RamseteCommand runPath(Trajectory path){
        //Find the initial pose of the trajectory, and set the to the robot position
        Pose2d initialPose = path.getInitialPose();

        driveSubsystem.zeroSensors();
        driveSubsystem.differentialDriveOdometry.resetPosition(
            new Rotation2d(0), 
            0, 
            0, 
            initialPose);

        RamseteCommand ramseteCommand = new RamseteCommand
        (path, 
        driveSubsystem::getPose, 
        new RamseteController(), 
        new SimpleMotorFeedforward(
            Constants.DriveConstants.kDriveKS, 
            Constants.DriveConstants.kDriveKS, 
            Constants.DriveConstants.kDriveKS), 
        driveSubsystem.differentialDriveKinematics, 
        driveSubsystem::getWheelSpeeds, 
        new PIDController(
            Constants.DriveConstants.kDrivekP, 
            Constants.DriveConstants.kDrivekI, 
            Constants.DriveConstants.kDrivekD
            ), 
            new PIDController(
                Constants.DriveConstants.kDrivekP, 
                Constants.DriveConstants.kDrivekI, 
                Constants.DriveConstants.kDrivekD
                ), 
        driveSubsystem::setVoltage, 
        driveSubsystem);

        return ramseteCommand;



    }
}
