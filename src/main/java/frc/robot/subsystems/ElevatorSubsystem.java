//PURPOSE
//This subsystem needs to encompass all CAN objects on the Elevator 
//This has one falcon motor.


//TODO:
//      Find out what motor type is here    
//      Define motors

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}