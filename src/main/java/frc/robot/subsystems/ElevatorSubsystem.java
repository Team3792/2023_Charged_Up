//PURPOSE
//This subsystem needs to encompass all CAN objects on the Elevator 
//This has one falcon motor.


//TODO:
//      Find out what motor type is here    
//      Define motors

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.UnsatisfiedLinkError;

public class ElevatorSubsystem extends SubsystemBase {
  //CHANGE THIS ONCE WE KNOW WHAT MOTOR CONTROLLER WE'RE USING
  //Do we need feedforward? I don't think so
   private TalonFX elevatorMotor = new TalonFX(Constants.MotorID.kElevatorMotor);
   private PIDController elevatorPID = new PIDController
  (
  Constants.ElevatorConstants.kElevatorkP, 
  Constants.ElevatorConstants.kElevatorkI, 
  Constants.ElevatorConstants.kElevatorkD
  );

  public ElevatorSubsystem(){

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