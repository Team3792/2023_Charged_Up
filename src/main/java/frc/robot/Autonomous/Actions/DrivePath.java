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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    public SequentialCommandGroup runPath(Trajectory path){

        RamseteCommand ramseteCommand =
            new RamseteCommand(
             path,
                driveSubsystem::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(
                 0.22,
                 1.98,
                 0.2),
             driveSubsystem.differentialDriveKinematics,
                driveSubsystem::getWheelSpeeds,
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
        // RamseteCommand passes volts to the callback
                driveSubsystem::setVoltage,
                driveSubsystem);

// Reset odometry to the starting pose of the trajectory.
driveSubsystem.resetOdometry(path.getInitialPose());

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> driveSubsystem.stopAndBreak());
    }
}
