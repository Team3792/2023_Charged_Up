//PURPOSE
//This subsystem needs to encompass all CAN objects on the Boom (AKA the boom)
//This will have one motor (unknown type, ask CAD)

//We don't need the motor type, we need the motor controller, ask electrical/Dr. Andy


//TODO:
//      Find out what motor type is here    
//      Define motors
   

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;

import org.apache.commons.collections4.functors.ConstantFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class BoomSubsystem extends SubsystemBase {
  //Assuming Talon_FX
  public WPI_TalonFX boomMotor = new WPI_TalonFX(Constants.MotorID.kBoomMotor);

  PIDController boomPidController = new PIDController(
    Constants.BoomConstants.kBoomkP, 
    Constants.BoomConstants.kBoomkI, 
    Constants.BoomConstants.kBoomkD
    );



    

  public BoomSubsystem() {
   boomMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   boomMotor.setSelectedSensorPosition(0);
   boomMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setPosition(double encoderTicks){

    // doneMoving = 
    // Math.abs(boomMotor.getSelectedSensorPosition() - encoderTicks) < Constants.BoomConstants.kStopDeadzone
    // &&
    // Math.abs(boomMotor.getSelectedSensorVelocity()) < Constants.BoomConstants.kStopVelocityMax;
    // if(!doneMoving){
    // double output = boomPidController.calculate(boomMotor.getSelectedSensorPosition(),encoderTicks);
    // boomMotor.setVoltage(output);
    // }else{
    //   boomMotor.setVoltage(0);
     
    // }
 }
//this method takes care of the endpoint deadbands and outputs the voltage to set the motor to
 public double getSetVoltage(double voltage){

  //by default, set the voltage to the un edited voltage
  //this will be used if the position is in the cruise zone

  double outputVoltage = voltage;

 double positionTicks = boomMotor.getSelectedSensorPosition();

 if(voltage > 0){ //if going forward
  //first check if beyond the max reach, then, if not, check if in the creep zone
  if(positionTicks >= Constants.BoomConstants.kBooomMaxReach){
    outputVoltage = 0;
  }else if(positionTicks > Constants.BoomConstants.kBooomMaxReach - Constants.BoomConstants.kBoomCreepRadius){ //if in the creep zone on the extended phase
    outputVoltage = Constants.BoomConstants.kCreepVoltage; //use the creep voltage if in this zone;
  }
 }else if(voltage < 0){ //if going backward
  //first check if beyond the max contractions, then, if not, check if in the creep zone
  if(positionTicks <= 0){
    outputVoltage = 0;
  }else if(positionTicks < 0 + Constants.BoomConstants.kBoomCreepRadius){ //if in the creep zone on the contraction phase
    outputVoltage = Constants.BoomConstants.kCreepVoltage; //use the creep voltage if in this zone;
  }
 }
 return outputVoltage;
 }

 public void setVoltage(double voltage){
  double editedVoltage = getSetVoltage(voltage);
  boomMotor.setVoltage(editedVoltage);
}

 public void setPositionDistance(double distanceMeters){
  
 }

 public double getPosition(){
  return boomMotor.getSelectedSensorPosition();
 }

 //this method returns whether or not the boom is within the contracted deadband
 public boolean isContracted(){
  return boomMotor.getSelectedSensorPosition() < Constants.BoomConstants.kContractedRadius;
 }


  @Override
  public void periodic() {

    //System.out.println(boomMotor.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}