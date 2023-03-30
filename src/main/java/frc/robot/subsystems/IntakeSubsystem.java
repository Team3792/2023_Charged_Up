//PURPOSE
//This subsystem needs to encompass all CAN objects on the Intake 
//This has one motor (uknown type, ask CAD)


//TODO:
//      Find out what motor type is here    
//      Define motors

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class IntakeSubsystem extends SubsystemBase {

  public WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.MotorID.kIntakeMotor);

  //Calculate how much the slew rate limiter should go up each second, based on desired voltage and desired time
  //Absolute value is included to prevent sign mistakes/differing setups
  public SlewRateLimiter slewRateLimiter = new SlewRateLimiter(
    Math.abs(Constants.IntakeConstants.kConeIntakeVoltage) / Constants.IntakeConstants.kIntakeRampTime, 
    -Math.abs(Constants.IntakeConstants.kConeIntakeVoltage) / Constants.IntakeConstants.kIntakeRampTime, 
    0);
    
  public IntakeSubsystem() {
    //When the intake stops, we don't want it to let the cone/cube go, so we break it
   intakeMotor.setNeutralMode(NeutralMode.Brake);

    
  }

  

  public void cubeIntake(){
    double output =  slewRateLimiter.calculate(Constants.IntakeConstants.kCubeIntakeVoltage);

    intakeMotor.setVoltage(output); ;
    System.out.println("this is the output" + output);
  }


  public void coneIntake(){
    double output =  slewRateLimiter.calculate(Constants.IntakeConstants.kConeIntakeVoltage);
    intakeMotor.setVoltage(output); 
   
  }

  public void cubeExtake(){
    intakeMotor.setVoltage(Constants.IntakeConstants.kCubeExtakeVoltage); 
  }

  public void coneExtake(){
    intakeMotor.setVoltage(Constants.IntakeConstants.kConeExtakeVoltage); 
  }

  public void stopIntake(){
intakeMotor.setVoltage(0);

      
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