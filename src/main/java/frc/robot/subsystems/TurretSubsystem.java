//PURPOSE
//This subsystem needs to encompass all CAN objects on the Turret
//This has one 775 pro motor


//TODO: Build some optimization code so the turret never rotates fully around;



package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;



public class TurretSubsystem extends SubsystemBase {
  //assumIng talonFX, CORRECT LATER
  public WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.MotorID.kTurretMotor);

  PIDController turretPidController = new PIDController(
    0.01, 
    Constants.TurretConstants.kTurretkI, 
    Constants.TurretConstants.kTurretkD);

    public double addOn = 0;

   public boolean doneMoving = true;

  public TurretSubsystem() {

    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    turretMotor.setSelectedSensorPosition(0);
    
  }



  public void setPosition(double degrees){
   // double addOn = (moved == true)? -180:0;
    double setPointTicks = degreesToTicks(degrees + addOn);
System.out.println(turretMotor.getSelectedSensorPosition() - setPointTicks);
    doneMoving = 
      Math.abs(turretMotor.getSelectedSensorPosition() - setPointTicks) < 100;


    if(!doneMoving){
    

      double output = turretPidController.calculate(turretMotor.getSelectedSensorPosition(), setPointTicks);
     System.out.println(turretMotor.getSelectedSensorPosition());
      //Should we set PIDs with voltage instead?
      turretMotor.setVoltage(-output);
    }else{
      turretMotor.setVoltage(0);
    }
  }

  public double getAngleDegrees(){
   // System.out.println(ticksToDegrees(turretMotor.getSelectedSensorPosition()));
    return ticksToDegrees(turretMotor.getSelectedSensorPosition());
  }

  private double ticksToDegrees(double ticks){
    double rotationsTurret = ticks/4096;
    double angleDegrees = rotationsTurret * 360;
    return angleDegrees;
  }

  private double degreesToTicks(double degrees){
    //Assuming 20248 ticks per rev
    double rotationsTurret = degrees / 360;
    //The encoder on the turret already takes the gear ratio into account
    double ticks = rotationsTurret * 4096;

    return ticks;
  }

  @Override
  public void periodic() {
   // System.out.println(turretMotor.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}