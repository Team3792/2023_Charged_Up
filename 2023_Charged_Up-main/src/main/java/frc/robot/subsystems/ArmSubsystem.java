//PURPOSE
//This subsystem needs to encompass all CAN objects on the arm (AKA the boom)
//This will have one motor (unknown type, ask CAD)


//TODO:
//      Find out what motor type is here    
//      Define motors
   

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}