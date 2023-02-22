//PURPOSE
//This subsystem needs to encompass all CAN objects on the Elevator 
//This has one falcon motor.


//TODO:
//      Find out what motor type is here    
//      Define motors

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class ElevatorSubsystem extends SubsystemBase {
  //CHANGE THIS ONCE WE KNOW WHAT MOTOR CONTROLLER WE'RE USING
  //Do we need feedforward? I don't think so
   private WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(Constants.MotorID.kElevatorMotor);
   private PIDController elevatorPID = new PIDController
  (
  Constants.ElevatorConstants.kElevatorkP, 
  Constants.ElevatorConstants.kElevatorkI, 
  Constants.ElevatorConstants.kElevatorkD
  );

  public ElevatorSubsystem(){
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    elevatorMotor.setSelectedSensorPosition(0);
    //Add code to zero sensors.
  }
//Later, we can do these in terms of heights, instead of ticks

 public void setPosition(double encoderTicks){
    double output = elevatorPID.calculate(elevatorMotor.getSelectedSensorPosition(),encoderTicks);
    elevatorMotor.set(ControlMode.PercentOutput, output);
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