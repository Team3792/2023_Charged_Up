//PURPOSE
//This subsystem needs to encompass all CAN objects on the Turret
//This has one 775 pro motor


//TODO: Build some optimization code so the turret never rotates fully around;



package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;



public class TurretSubsystem extends SubsystemBase {
  //assumIng talonFX, CORRECT LATER
  WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.MotorID.kTurretMotor);

  PIDController turretPidController = new PIDController(
    Constants.TurretConstants.kTurretkP, 
    Constants.TurretConstants.kTurretkI, 
    Constants.TurretConstants.kTurretkD);

  public TurretSubsystem() {

    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    turretMotor.setSelectedSensorPosition(0);
    
  }

  public void setPosition(double degrees){
    double setPointTicks = degreesToTicks(degrees);

      double output = turretPidController.calculate(turretMotor.getSelectedSensorPosition(), setPointTicks);
      //Should we set PIDs with voltage instead?
      turretMotor.setVoltage(output);
  }

  private double getAngleDegrees(){
    return ticksToDegrees(turretMotor.getSelectedSensorPosition());
  }

  private double ticksToDegrees(double ticks){
    double rotationsTurret = ticks/2048;
    double angleDegrees = rotationsTurret * 360;
    return angleDegrees;
  }

  private double degreesToTicks(double degrees){
    //Assuming 20248 ticks per rev
    double rotationsTurret = degrees / 360;
    //The encoder on the turret already takes the gear ratio into account
    double ticks = rotationsTurret * 2048;

    return ticks;
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