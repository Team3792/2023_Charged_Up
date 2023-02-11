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
      public static final int kLeftDriveFollowMotor = 1;
      public static final int kRightDriveLeadMotor = 2;
      public static final int kRightDriveFollowMotor = 3;


      //Gyro
      public static final int kGyro = 8;

      //Elevator
      public static final int kElevatorMotor = 7;

      //Turret (Falcon SRX)
      public static final int kTurretMotor = 4;

      //Arm/Intake
      public static final int kBoomMotor = 5;
      //public static final int kArmMotor2 = 0;  //might not exist
      public static final int kIntakeMotor = 6;


    }

  public static final class BoomConstants
  {

    public static final double kBoomDeadband = 0;
    
    //PID constants for position control

    public static final double kBoomkP = 0;
    public static final double kBoomkI = 0;
    public static final double kBoomkD = 0;


    //For now, this is interpreted as being in encoder ticks. We might want to make a conversion function later if it helps with the math of positioning or other things
    public static final double kBooomMaxReach = 0;
  }



  public static final class DriveConstants
    {
      //Drive Kinematics
      public static final double kDriveTrainWidthMeters = 0.5842;
    
      //Drivetrain PID Vars
      public static final double kDrivekP = 0;
      public static final double kDrivekI = 0;
      public static final double kDrivekD = 0;

      //Drivetrina Feedforward

      public static final double kDriveKS = 0;
      public static final double kDriveKV = 0;
      public static final double kDriveKA = 0;
      //Drive Speed Constants
      public static final double kMaxDriveSpeed = 0;           // meters per second
      public static final double kMaxDriveAngularSpeed = 0; //rotations per second

      //Deadzone
      public static final int kDriveDeadzone = 0;
    }

  public static final class ElevatorConstants
    {
      //Elevator PID Vars
      public static final double kElevatorkP = 0;
      public static final double kElevatorkI = 0;
      public static final double kElevatorkD = 0;

      //Elevator Speed Constants
      public static final double kMaxElevatorSpeed = 0;     //meters per second

      //Deadzone
      public static final int kElevatorDeadzone = 0;

      //Heights for placing, in ENCODER TICKS
      public static final int kCubeHighHeight = 0;
      public static final int kCubeMiddleHeight = 0;
      public static final int kCubeGroundHeight = 0;

      public static final int kConeHighHeight = 0;
      public static final int kConeMiddleHeight = 0;
      public static final int kConeGroundHeight = 0;

    }

  public static final class LEDConstants
  {
    //DIO channels
    public static final int kLEDChannel0 = 0;
    public static final int kLEDChannel1 = 0;
    public static final int kLEDChannel2 = 0;
  }

  public static final class TurretConstants
    {
      //TurretConstants PID Vars
      public static final double kTurretkP = 0;
      public static final double kTurretkI = 0;
      public static final double kTurretkD = 0;

      //Turret Speed Constants
      public static final double kMaxTurretSpeed = 0;
      public static final double kMaxTurretAngularSpeed = 0;     //rotations per second

      //Deadzone
      public static final int kTurretDeadzone = 0;

      //Maximum turret angle, where 0 is straight ahead. (this is for both sides)
      public static final double kMaxTurretAngle = 0;
      
    }

  public static final class IntakeConstants
    {
      //Setting intake and extake (I know that's not a word) velocities
      //Notice that if we need to spin backward, 
      public static final double kCubeIntakeVelocity = 0;
      public static final double kConeIntakeVelocity = 0;
      public static final double kCubeExtakeelocity = 0;
      public static final double kConeExtakeVelocity = 0;


    }

}

