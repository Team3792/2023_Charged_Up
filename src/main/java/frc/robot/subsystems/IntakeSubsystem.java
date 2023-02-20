//PURPOSE
//This subsystem needs to encompass all CAN objects on the Intake 
//This has one motor (uknown type, ask CAD)


//TODO:
//      Find out what motor type is here    
//      Define motors

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(Constants.MotorID.kIntakeMotor);

  public IntakeSubsystem() {
    //When the intake stops, we don't want it to let the cone/cube go, so we break it
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void cubeIntake(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.kCubeIntakeVelocity); 
  }

  public void coneIntake(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.kConeIntakeVelocity); 
  }

  public void cubeExtake(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.kCubeExtakeelocity); 
  }

  public void coneExtake(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.kConeExtakeVelocity); 
  }

  public void stopIntake(){

    intakeMotor.set(ControlMode.PercentOutput, 0);
      
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