//PURPOSE
//This subsystem needs to encompass all CAN objects on the Boom (AKA the boom)
//This will have one motor (unknown type, ask CAD)

//We don't need the motor type, we need the motor controller, ask electrical/Dr. Andy


//TODO:
//      Find out what motor type is here    
//      Define motors
   

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class BoomSubsystem extends SubsystemBase {
  //Assuming Talon_FX
  private TalonSRX boomMotor = new TalonSRX(Constants.MotorID.kBoomMotor);

  PIDController boomPidController = new PIDController(
    Constants.BoomConstants.kBoomkP, 
    Constants.BoomConstants.kBoomkI, 
    Constants.BoomConstants.kBoomkD
    );

  public BoomSubsystem() {
   boomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
   boomMotor.setSelectedSensorPosition(0);
  }

  public void setPosition(double encoderTicks){
    double output = boomPidController.calculate(boomMotor.getSelectedSensorPosition(),encoderTicks);
    boomMotor.set(ControlMode.PercentOutput, output);
 }

 public void setPositionDistance(double distanceMeters){
  
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