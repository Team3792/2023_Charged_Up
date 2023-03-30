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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  //Creating gyro
  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.MotorID.kGyro);

  // Motor Controller Groups 
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLead, rightFollow);

  //Differential drive objects
  public final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters);
  public final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  public DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(
      new Rotation2d(0),
      0, 
      0);


 



  public DriveSubsystem() {
    //Use this section to set up motor attributes

    


  

    //Inverting right motors

   rightMotors.setInverted(true);
   // setNeutral(NeutralMode.Coast);
    //Configure feedback encoder
    rightLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    zeroSensors();

 


  }

  

  public void zeroSensors(){
    leftLead.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);

    pigeon.reset();
  }



 

  

 

  public void setVoltage(double leftVoltage, double rightVoltage){
    //By setting the right motors inverted, we do not need to negative rightVoltage
    //IMPORTANT: make sure to put power to both motors on each side

    leftMotors.setVoltage(leftVoltage);
    rightMotors.setVoltage(rightVoltage);

  }

  public double getTotalDistance(){
    //return average distance traveled by each wheel
    return Math.abs((toMeters(-rightLead.getSelectedSensorPosition()) + toMeters(leftLead.getSelectedSensorPosition()))/2);
  }
  
  //returns the pitch of the gryo
  public double getPitch(){
    return pigeon.getPitch();
  }

  public double toMeters(double ticks){
    double motorRotations = ticks/2048;
    double wheelRotations = motorRotations/Constants.RobotDimensionConstants.kMotorToWheelShaftGearRatio;
    double distancePerRotationMeters = Math.PI*Units.inchesToMeters(Constants.RobotDimensionConstants.kWheelDiameterInches);
    double meters = distancePerRotationMeters * wheelRotations;
    return meters;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    //Negative on the right because, though it was inverted, its encoder was not
    //Multiplying by 10 because getSelectedSensorVelocity return rotationas per 100ms, or .1 sec
    double rightMetersPerSecond = -toMeters(rightLead.getSelectedSensorVelocity()*10.0);
    double leftMetersPerSecond = toMeters(leftLead.getSelectedSensorVelocity()*10.0);

    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

//Use this method to stop the bot and break it, used for auto and charge station balanceing
  public void stopAndBreak(){
    setNeutral(NeutralMode.Brake);
    setVoltage(0, 0);
  }
//This provides a simple way to set the neutral mode on every motor at once
  public void setNeutral(NeutralMode neutralMode){
    rightLead.setNeutralMode(neutralMode);
    leftLead.setNeutralMode(neutralMode);
    rightFollow.setNeutralMode(neutralMode);
    leftFollow.setNeutralMode(neutralMode);
  }

  @Override
  public void periodic() {
   // This method will be called once per schegduler run
   //Update the odemetry with encoder (converted) and pigeon data
    differentialDriveOdometry.update(
      pigeon.getRotation2d(), 
      toMeters(leftLead.getSelectedSensorPosition()), 
      toMeters(-rightLead.getSelectedSensorPosition())
      );


 

  }

  public Pose2d getPose(){
    //return the robot pose from odeometry
    return differentialDriveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d resetPose){
    differentialDriveOdometry.resetPosition(
    pigeon.getRotation2d(),
    toMeters(leftLead.getSelectedSensorPosition()), 
    toMeters(-rightLead.getSelectedSensorPosition()), 
    resetPose);

    

  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
