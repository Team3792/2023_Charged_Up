//PURPOSE
//This subsystem needs to encompass the method in which the RoboRio communicates with the Arduino
//This will end up affecting the LED colors on the robot


//TODO:
//      Figure out how Rio --> arduino communication works   - Done, see https://docs.wpilib.org/en/stable/docs/hardware/sensors/digital-inputs-hardware.html  
//      Start setting up lines of communication for them - All done!!!!

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  //0 - ones
  //1 - twos
  //2 - fours
  DigitalOutput digitalOutput0 = new DigitalOutput(Constants.LEDConstants.kLEDChannel0);
  DigitalOutput digitalOutput1 = new DigitalOutput(Constants.LEDConstants.kLEDChannel1);
  DigitalOutput digitalOutput2 = new DigitalOutput(Constants.LEDConstants.kLEDChannel2);

  public LEDSubsystem() {}

  public void sendCode(int decimalProgram){
    //Converting decimalProgram into binary digits
    int onesDigit = decimalProgram % 2;
    int twosDigit = (decimalProgram % 4 - onesDigit)/2;
    int foursDigit = (decimalProgram % 8 - twosDigit)/4;

    digitalOutput0.set((onesDigit == 1)? true:false);
    digitalOutput1.set((twosDigit == 1)? true:false);
    digitalOutput2.set((foursDigit == 1)? true:false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
