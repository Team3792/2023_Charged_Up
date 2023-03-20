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
import com.ctre.phoenix.motorcontrol.NeutralMode;



public class TurretSubsystem extends SubsystemBase {
  //assumIng talonFX, CORRECT LATER
  public WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.MotorID.kTurretMotor);

  PIDController turretPidController = new PIDController(
    0.01, 
    Constants.TurretConstants.kTurretkI, 
    Constants.TurretConstants.kTurretkD);

    public double relativeMark = 0;
    public double setPointDegrees = 0;

    //booleans for arrived (move on to next action in auto/an auto sequence)
    public boolean arrived = false;

  public TurretSubsystem() {

    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    turretMotor.setSelectedSensorPosition(0);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    
  }

  public void checkArrival(){
    arrived = Math.abs(turretMotor.getSelectedSensorPosition() - setPointDegrees) < Constants.TurretConstants.kTurretArrivedDeadzone;
  }

  public void setSetPointDegrees(double setPointDegrees){
    this.setPointDegrees = setPointDegrees;
  }

  public void setRelativeMark(double relativeMark){
    this.relativeMark = relativeMark;
  }
//lockToLeft and lockToRight are used to turn the turret 90 deg left or right, respectively, to score sideways
//0 deg is turtle mode
//180 deg is intake mode
//ccw is positive
//For reference, picture the robot facing straight right

  public void lockToLeft(){
    setRelativeMark(270);
  }
  
  public void lockToRight(){
    setRelativeMark(90);

  }


  public void setPosition(){
  
    double setPointTicks = degreesToTicks(setPointDegrees + relativeMark);

    


    
    
    //Calculate and set PID voltage
      double output = turretPidController.calculate(turretMotor.getSelectedSensorPosition(), setPointTicks);
     
      
      turretMotor.setVoltage(-output);
    
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
    //Constantly check if the turret has arrived (used for sequencing), and update the volage to achive the wanted angle
    checkArrival();
    setPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}