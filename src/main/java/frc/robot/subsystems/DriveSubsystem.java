//PURPOSE
//This subsystem is the PID drive subsystem. Most PID is in here
//This has a drive train with 4 falcon motors


//TODO:
//      Find out exact drive train details
//      possibly change PID to the command and not the subsystem

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

  //Defining drive motors
  public final WPI_TalonFX leftLead = new WPI_TalonFX(Constants.MotorID.kLeftDriveLeadMotor);
  private final WPI_TalonFX leftFollow = new WPI_TalonFX(Constants.MotorID.kLeftDriveFollowMotor);
  public final WPI_TalonFX rightLead = new WPI_TalonFX(Constants.MotorID.kRightDriveLeadMotor);
  private final WPI_TalonFX rightFollow = new WPI_TalonFX(Constants.MotorID.kRightDriveFollowMotor);
  
  //Using a pigeon connectd to talon on Mantis for testing
 // private final WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(15);
 // public final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(pigeonTalon);

  // Motor Controller Groups 
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLead, rightFollow);

  //Differential drive objects
  public final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters);
  public final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  public DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(
    new Rotation2d(0),0, 0);
  public Pose2d robotPose = new Pose2d();

  

 // private W pigeonMotor  = new WPI_TalonFX(2);

 
 



  public DriveSubsystem() {
    //Use this section to set up motor attributes

    //Pairing lead and follow motors
    leftFollow.follow(leftLead);
    rightFollow.follow(rightLead);

    //Inverting right motors
   // leftMotors.setInverted(true);
    rightMotors.setInverted(true);
    setNeutral(NeutralMode.Coast);

    zeroSensors();

    //pigeonMotor.configSelectedFeedbackSensor(FeedbackDevice.)


  }

  

  public void zeroSensors(){
    leftLead.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);

   // pigeon.reset();
  }



 

  

 

  public void setVoltage(double leftVoltage, double rightVoltage){
    leftLead.setVoltage(leftVoltage);
    rightLead.setVoltage(rightVoltage);
    
  }

  public double getPitch(){
   // return pigeon.getPitch();
    return 0;
  }

  public void updatePigeonAngle(double angleDegrees){
  // pigeon.setYaw(angleDegrees);
  }

  public double toMeters(double ticks){
    double motorRotations = ticks/2048;
    //replace 10 with gear ratio
    double wheelRotations = motorRotations/Constants.RobotDimensionConstants.kMotorToWheelShaftGearRatio;
    double distancePerRotationMeters = Math.PI*Units.inchesToMeters(Constants.RobotDimensionConstants.kWheelDiameterInches);
    double meters = distancePerRotationMeters * wheelRotations;
    return meters;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    double rightMetersPerSecond = -toMeters(rightLead.getActiveTrajectoryVelocity()*10);
    double leftMetersPerSecond = toMeters(leftLead.getActiveTrajectoryVelocity()*10);
    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }
//Use this method to stop the bot and break it, used for auto and charge station balanceing
  public void stopAndBreak(){
    setNeutral(NeutralMode.Brake);
    differentialDrive.arcadeDrive(0, 0);
    
  }

  public void setNeutral(NeutralMode neutralMode){
    rightLead.setNeutralMode(neutralMode);
    leftLead.setNeutralMode(neutralMode);
    rightFollow.setNeutralMode(neutralMode);
    leftFollow.setNeutralMode(neutralMode);
  }

  @Override
  public void periodic() {

    robotPose = differentialDriveOdometry.update(
    //  pigeon.getRotation2d(), 
    new Rotation2d(0),
      toMeters(leftLead.getSelectedSensorPosition()), 
      toMeters(-rightLead.getSelectedSensorPosition())
      );

     // System.out.println(pigeon.getRotation2d().getDegrees());
    // This method will be called once per scheduler run

  }

  public Pose2d getPose(){
    return robotPose;
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
