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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class ElevatorSubsystem extends SubsystemBase {

  //Do we need feedforward? I don't think so
   public WPI_TalonFX elevatorMotor = new WPI_TalonFX(Constants.MotorID.kElevatorMotor);
   private PIDController elevatorPID = new PIDController
  (
  Constants.ElevatorConstants.kElevatorkP, 
  Constants.ElevatorConstants.kElevatorkI, 
  Constants.ElevatorConstants.kElevatorkD
  );

  public ElevatorSubsystem(){
    //elevatorMotor.setSelectedSensorPosition(0);
    //zero sensors, this should be done at buttom,
  elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }
//Later, we can do these in terms of heights, instead of ticks

 public void setPosition(double encoderTicks){
    double output = elevatorPID.calculate(elevatorMotor.getSelectedSensorPosition(),encoderTicks);
    elevatorMotor.setVoltage(output * 12);
    //elevatorMotor.set(ControlMode.PercentOutput, output);
 }

 //This is a temperary method used to test elevator and find setpoints
 public void setVoltage(double voltage){
  elevatorMotor.setVoltage(voltage);
  System.out.println(elevatorMotor.getSelectedSensorPosition());

  //Put ticks on smart dashboard to determine setpoints
  //SmartDashboard.putNumb
 }



  @Override
  public void periodic() {
   // er("volts", voltage);
 
  //SmartDashboard.putNumber("elevator encoder ticks", elevatorMotor.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}