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

    public boolean doneMoving = false;

  public BoomSubsystem() {
   boomMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   boomMotor.setSelectedSensorPosition(0);
   boomMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setPosition(double encoderTicks){

    doneMoving = 
    Math.abs(boomMotor.getSelectedSensorPosition() - encoderTicks) < Constants.BoomConstants.kStopDeadzone
    &&
    Math.abs(boomMotor.getSelectedSensorVelocity()) < Constants.BoomConstants.kStopVelocityMax;
    if(!doneMoving){
    double output = boomPidController.calculate(boomMotor.getSelectedSensorPosition(),encoderTicks);
    boomMotor.setVoltage(output);
    }else{
      boomMotor.setVoltage(0);
     
    }
 }

 public void setVoltage(double voltage){

 // Sys
 if(!(voltage < 0 && boomMotor.getSelectedSensorPosition() < 0) && !(voltage > 0 && boomMotor.getSelectedSensorPosition() > 200000)){
  doneMoving = false;
    if(!(voltage < 0 && boomMotor.getSelectedSensorPosition() < 20000) && !(voltage > 0 && boomMotor.getSelectedSensorPosition() > 180000)){
      boomMotor.setVoltage(voltage);
    }
    else{
      if(voltage>0){
      boomMotor.setVoltage(1.5);
    }else if(voltage < 0){
      boomMotor.setVoltage(-1.5);
    }
  }
  
 }else {
  boomMotor.setVoltage(0);
  doneMoving = true;
 }
}

 public void setPositionDistance(double distanceMeters){
  
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