//PURPOSE
//This subsystem needs to encompass all CAN objects on the Elevator 
//This has one falcon motor.


//TODO:
//      Find out what motor type is here    
//      Define motors

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class ElevatorSubsystem extends SubsystemBase {

  
   public WPI_TalonFX elevatorMotor = new WPI_TalonFX(Constants.MotorID.kElevatorMotor);

   private PIDController elevatorPID = new PIDController
  (
    Constants.ElevatorConstants.kElevatorkP, 
    Constants.ElevatorConstants.kElevatorkI, 
    Constants.ElevatorConstants.kElevatorkD
  );

  public boolean arrived = false;
  public double setPoint = 0;
  public boolean manualMode = false; //this is used to determine whether or not to run the PID in the periodic function

  public ElevatorSubsystem(){
    //elevatorMotor.setSelectedSensorPosition(0);
    //zero sensors, this should be done at buttom,
  elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

  //Set the elevatorMotor to neutral mode so it breaks when there is 0 voltage
  elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }
//Later, we can do these in terms of heights, instead of ticks

public void checkArrival(){
   //If the elevator is within a certain encoder distance and slow enough speed, set arrived to true
   arrived = 
   Math.abs(elevatorMotor.getSelectedSensorPosition() - setPoint) < Constants.ElevatorConstants.kArrivedDeadzone;
  //  &&
  //  Math.abs(elevatorMotor.getSelectedSensorVelocity()) < Constants.ElevatorConstants.kStopVelocityMax;
}

 public void setPosition(){
 
  double currentPositionEncoderTicks = elevatorMotor.getSelectedSensorPosition();
  
  //If within the stop (and break) dead zone, stop, other wise, keep giving voltage;
    if(Math.abs(setPoint - currentPositionEncoderTicks) < Constants.ElevatorConstants.kStopDeadzone){
     
      setVoltage(0);
   
    }else{

    double output = elevatorPID.calculate(currentPositionEncoderTicks, setPoint);
    setVoltage(output);

    }


   
}

 //This is a temperary method used to test elevator and find setpoints
 public void setVoltage(double voltage){
  //Check if voltage is out of the max voltage bound, and put it there if so
  if(voltage > Constants.ElevatorConstants.kMaxVoltage){
    elevatorMotor.setVoltage(Constants.ElevatorConstants.kMaxVoltage);
 }else if (voltage < -Constants.ElevatorConstants.kMaxVoltage){
  elevatorMotor.setVoltage(-Constants.ElevatorConstants.kMaxVoltage);
 }else {
  elevatorMotor.setVoltage(voltage);
 }
 }

 public void setSetPoint(double setPoint){
  this.setPoint = setPoint;
  manualMode = false;//we are auto moving!
  //Reset the PID controller, as there could have been a break for manual control
  elevatorPID.reset();
 }

 public void manualUp(){
  setVoltage(Constants.ElevatorConstants.kManualVoltage);
  engageManualMode();
 }

 public void manualDown(){
  setVoltage(-Constants.ElevatorConstants.kManualVoltage);
  engageManualMode();
  
 }
//This sets up the 
 public void engageManualMode(){
  manualMode = true;
 }

 public void stop(){
  setVoltage(0);
 }



  @Override
  public void periodic() {

 
  
    // This method will be called once per scheduler run
    //Constantly update motor and PID with set point
    checkArrival();
    //Only use this auto setPostition method if manualMode is false
    if(!manualMode){
    setPosition();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}