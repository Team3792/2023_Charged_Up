// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


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

      //Intake 
      public static final int kConeIntakeButton = 5;
      public static final int kCubeIntakeButton = 3;
      public static final int kDropAllButton = 1;
    
      //Elevator
    
      public static final int kGroundElevatorButton = 9;
      public static final int kMiddleElevatorButton = 11;
      public static final int kHighElevatorButton = 12;
    
      //Intake Elevator buttons
    
      public static final int kConeIntakeHigh = 6;
      public static final int kConeIntakeLow = 4;

      //Flipper toggle buttons

      public static final int kFlipperToggleButton = 0;

    }

  public static final class MotorID
    {
      //Drive Train (Talon FX/Falcon)
      public static final int kLeftDriveLeadMotor = 10;
      public static final int kLeftDriveFollowMotor = 11;
      public static final int kRightDriveLeadMotor = 12;
      public static final int kRightDriveFollowMotor = 13;


      //Gyro
      public static final int kGyro = 15;

      //Elevator
      public static final int kElevatorMotor = 20;

      //Turret
      public static final int kTurretMotor = 30;

      //Arm/Intake
      public static final int kBoomMotor = 40;
      //public static final int kArmMotor2 = 0;  //might not exist
      public static final int kIntakeMotor = 50;

      public static final int kFlipperMotor = 60;

      //PDH board

     // public static final int kPowerDistribution = 50;

    }

  public static final class PowerDistributionHubConstants
  {
    public static final int kPDHIntakeChannel = 12;
    
    //Limiting currents for intake

    public static final double kCubeShutoffCurrent = 15;
    public static final double kConeShutoffCurrent = 15;
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
    public static final double kBoomIntakeReach = 0;
  }



  public static final class DriveConstants
    {
      //Drive Kinematics
      public static final double kDriveTrainWidthMeters = 0.72;
    
      //Drivetrain PID Vars
      public static final double kDrivekP = 2.4;
      public static final double kDrivekI = 0;
      public static final double kDrivekD = 0;

      //Drivetrina Feedforward

      public static final double kDriveKS = 0;
      public static final double kDriveKV = 0;
      public static final double kDriveKA = 0;
      //Drive Speed Constants
      public static final double kMaxDriveSpeed = 5;           // meters per second
      public static final double kMaxDriveAngularSpeed = 0; //rotations per second

      //Deadzone
      public static final int kDriveDeadzone = 0;
    }

  public static final class ElevatorConstants
    {
      //Elevator PID Vars
      public static final double kElevatorkP = 0.001;
      public static final double kElevatorkI = 0;
      public static final double kElevatorkD = 0;

      //Elevator Speed Constants
      public static final double kMaxElevatorSpeed = 0;     //meters per second

      //Deadzone
      public static final int kElevatorDeadzone = 0;

      //Heights for placing, in ENCODER TICKS
      public static final int kCubeHighHeight = 0;
      public static final int kCubeMiddleHeight = 1000;
      public static final int kCubeGroundHeight = 0;

      public static final int kConeHighHeight = 0;
      public static final int kConeMiddleHeight = 0;
      public static final int kConeGroundHeight = 0;

      //Intake heights
      public static final double kConeHighIntake = 0;
      public static final double kCubeHighIntake = 0;
      public static final double kConeLowIntake = 0;
      public static final double kCubeLowIntake = 0;

    }

  public static final class LEDConstants
  {
    //DIO channels 
    //These are the channels that the LEDs are connected to
    //The LEDs are connected to the DIO ports on the RoboRIO
    public static final int kLEDChannel0 = 0;
    public static final int kLEDChannel1 = 1;
    public static final int kLEDChannel2 = 2;
    public static final int kLEDChannel3 = 3;
    
    //public static final int kLEDChannelHumanPlayer = 5;
    //public static final int kLEDChannelHUmanPlayer2 = 6;
  }

  public static final class TurretConstants
    {
      //TurretConstants PID Vars
      public static final double kTurretkP = 0.01;
      public static final double kTurretkI = 0;
      public static final double kTurretkD = 0;

      //Turret Speed Constants
      public static final double kMaxTurretSpeed = 3;
      public static final double kMaxTurretAngularSpeed = 0;     //rotations per second

      //Deadzone
      public static final int kTurretDeadzone = 0;

      //Maximum turret angle, where 0 is straight ahead. (this is for both sides)
      public static final double kMaxTurretAngle = 60;

  
      
    }

  public static final class IntakeConstants
    {
      //Setting intake and extake (I know that's not a word) velocities
      //Notice that if we need to spin backward, 
      public static final double kCubeIntakeVoltage = 8;
      public static final double kConeIntakeVoltage = -8;
      public static final double kCubeExtakeVoltage = -8;
      public static final double kConeExtakeVoltage = 8;

      //How many seconds it should take for the slew rate limiter to ramp up
      public static final double kIntakeRampTime = 0.5;


    }

  public static final class FieldLayoutConstants
  {

    /*Standard:
    Start with blue, top to bottom, ground node to high node. 
    For red, still go top to bottom, groun node to high node

    */

    //I know this is ugly:
    //Outside array is the three grids
    //Inside arrays are the individual spots
    
    public static final Pose2d[][] kRedCubeDropLocations = {
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      }
    };
    public static final Pose2d[][] kBlueCubeDropLocations = {
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      }
    };
    public static final Pose2d[][] kRedConeDropLocations = {
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      }
    };
    public static final Pose2d[][] kBlueConeDropLocations = {
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      },
      {
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
      }
      };


  }

  public static final class VisionCosntnats{

    public static double kAmbiguityThreshold = 0;

  }

  public static final class RobotDimensionConstants
  {
    public static double kWheelDiameterInches = 6;
    public static double kMotorToWheelShaftGearRatio = 7.62;
  }

  public static final class FlipperConstants
  {
    public static double kExtendVoltage = 4;
    public static double kRetractVoltage = -4;

  }

  public static final class ChargeStationStabilizeConstants
  {
    public static double kPIDkP = 1;
  }

}

