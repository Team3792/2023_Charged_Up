// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class DriveSubsystem extends SubsystemBase {

  //Defining drive motors
  private final WPI_TalonFX leftLead = new WPI_TalonFX(Constants.MotorID.kLeftDriveLeadMotor);
  private final WPI_TalonFX leftFollow = new WPI_TalonFX(Constants.MotorID.kLeftDriveFollowMotor);
  private final WPI_TalonFX rightLead = new WPI_TalonFX(Constants.MotorID.kRightDriveLeadMotor);
  private final WPI_TalonFX rightFollow = new WPI_TalonFX(Constants.MotorID.kRightDriveFollowMotor);

  // Motor Controller Groups 
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLead, rightFollow);

  //Differential drive objects
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters);


  //Creating gyro
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.MotorID.kGyro);

  //Defining PID and Feedforward controllers
 
  private final PIDController leftPIDController = new PIDController(
    Constants.DriveConstants.kDrivekP,
    Constants.DriveConstants.kDrivekI,
    Constants.DriveConstants.kDrivekD);

  private final PIDController rightPIDController = new PIDController(
    Constants.DriveConstants.kDrivekP,
    Constants.DriveConstants.kDrivekI,
    Constants.DriveConstants.kDrivekD);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.DriveConstants.kDriveKS, 
      Constants.DriveConstants.kDriveKV, 
      Constants.DriveConstants.kDriveKA);



  public DriveSubsystem() {
    //Use this section to set up motor attributes

    //Pairing lead and follow motors
    leftFollow.follow(leftLead);
    rightFollow.follow(rightLead);

    //Inverting right motors
    rightMotors.setInverted(true);

    zeroSensors();


  }

  private void zeroSensors(){
    leftLead.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);
  }

  public final void drive(double forward, double rotation){


    //Add deadband code later

    var wheelSpeeds = differentialDriveKinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0, rotation));
    setSpeeds(wheelSpeeds);

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    double leftSpeed = toMeters(leftLead.getSelectedSensorVelocity());
    double rightSpeed = -toMeters(rightLead.getSelectedSensorVelocity());

    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  public double toMeters(double ticksPerSecond){
    double motorRotationsPerSecond = ticksPerSecond/2048;
    //replace 10 with gear ratio
    double wheelRotationsPerSecond = motorRotationsPerSecond/10;
    double distancePerRotations = Math.PI*Units.inchesToMeters(6);
    double metersPerSecond = distancePerRotations * wheelRotationsPerSecond;
    return metersPerSecond;
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);

    DifferentialDriveWheelSpeeds currentWheelsSpeeds = getWheelSpeeds();
    double rightOutput = rightPIDController.calculate(
      currentWheelsSpeeds.rightMetersPerSecond, 
      speeds.rightMetersPerSecond) + rightFeedforward;

    double leftOutput = leftPIDController.calculate(
      currentWheelsSpeeds.leftMetersPerSecond, 
      speeds.leftMetersPerSecond) + leftFeedforward;

    setVolts(rightOutput, leftOutput);
  }

  private void setVolts(double leftVolts, double rightVolts){
    leftLead.setVoltage(leftVolts);
    rightLead.setVoltage(rightVolts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
