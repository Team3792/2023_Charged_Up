// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static final class ButtonConstant
    {
      //Driver Joystick
      public static final int kDriveJoystick = 0;
  


      //Operator Joystick
      public static final int kOperatorJoystick = 1;
    }

  public static final class MotorID
    {
      //Drive Train (Talon FX/Falcon)
      public static final int kLeftDriveLeadMotor = 0;
      public static final int kLeftDriveFollowMotor = 0;
      public static final int kRightDriveLeadMotor = 0;
      public static final int kRightDriveFollowMotor = 0;


      //Gyro
      public static final int kGyro = 0;

      //Elevator
      public static final int kElevatorMotor = 0;

      //Turret
      public static final int kTurretMotor = 0;

      //Arm/Intake
      public static final int kArmMotor1 = 0;
      public static final int kArmMotor2 = 0;  //might not exist


    }



  public static final class DriveConstants
    {
      //Drive Kinematics
      public static final double kDriveTrainWidthMeters = 0;
    
      //Drivetrain PID Vars
      public static final double kDrivekP = 0;
      public static final double kDrivekI = 0;
      public static final double kDrivekD = 0;
    
      //Drive Speed Constants
      public static final double kMaxDriveSpeed = 0;           // meters per second
      public static final double kMaxDriveAngularSpeed = 0; //rotations per second

      //Deadzone
      public static final int kDriveDeadzone = 0;
    }

  public static final class ElevatorConstants
    {
      //Elevator PID Vars
      public static final double kDrivekP = 0;
      public static final double kDrivekI = 0;
      public static final double kDrivekD = 0;

      //Elevator Speed Constants
      public static final double kMaxElevatorSpeed = 0;     //meters per second

      //Deadzone
      public static final int kElevatorDeadzone = 0;

    }

  public static final class TurretConstants
    {
      //TurretConstants PID Vars
      public static final double kDrivekP = 0;
      public static final double kDrivekI = 0;
      public static final double kDrivekD = 0;

      //Turret Speed Constants
      public static final double kMaxTurretSpeed = 0;
      public static final double kMaxTurretAngularSpeed = 0;     //rotations per second

      //Deadzone
      public static final int kTurretDeadzone = 0;
      
    }

}

