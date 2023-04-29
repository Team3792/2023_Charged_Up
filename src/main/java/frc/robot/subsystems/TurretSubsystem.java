//PURPOSE
//This subsystem needs to encompass all CAN objects on the Turret
//This has one 775 pro motor


//TODO: Build some optimization code so the turret never rotates fully around;



package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;



public class TurretSubsystem extends SubsystemBase {
  //assumIng talonFX, CORRECT LATER
  public WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.MotorID.kTurretMotor);

  PIDController turretPidController = new PIDController(
    Constants.TurretConstants.kTurretkP, 
    Constants.TurretConstants.kTurretkI, 
    Constants.TurretConstants.kTurretkD);

    PIDController turretPidControllerAbsolute = new PIDController(
    Constants.TurretConstants.kTurretkP, 
    Constants.TurretConstants.kTurretkI, 
    Constants.TurretConstants.kTurretkD);

    Timer timer = new Timer();
    double previousTime = 0;
    double previousSetpoint = 0;

    

    public double relativeMark = 0;
    public double setPointDegrees = 0;
    

    //booleans for arrived (move on to next action in auto/an auto sequence)
    public boolean arrived = false;
    public boolean locked = true;

    PIDController pid = new PIDController(0, 0, 0);
    PIDController pid2 = new PIDController(0, 0, 0);
    SlewRateLimiter srl = new SlewRateLimiter(100,-100,0);

  public TurretSubsystem() {

    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    turretMotor.setSelectedSensorPosition(0);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    timer.reset();
    timer.start();
    

    turretPidController.enableContinuousInput(-degreesToTicks(180), degreesToTicks(180));
    SmartDashboard.putNumber("Vel Constant", 0.01);
    SmartDashboard.putNumber("Yaw Vel Constant", 0.075);
    SmartDashboard.putNumber("TurretPID", 0.01);

  }

  public void checkArrival(){
    arrived = Math.abs(turretMotor.getSelectedSensorPosition() - degreesToTicks(setPointDegrees)) < Constants.TurretConstants.kTurretArrivedDeadzone;
    SmartDashboard.putBoolean("arrived", arrived);
  }

  public void setSetPointDegrees(double setPointDegrees){
    previousSetpoint = this.setPointDegrees;
    this.setPointDegrees = setPointDegrees;
    arrived = false;
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
    System.out.println(1);
    setRelativeMark(270);
    locked = false;
  }
  
  public void lockToRight(){
    setRelativeMark(90);
    locked = false;

  }

  public void lockToFront(){
    setRelativeMark(180);
    locked = false;
  }

  public void lockToBack(){ 
    setRelativeMark(0);
    locked = false;
  }


  public void setPosition(){
    double setPointTicks, output;
     output = 0;
    double velocity = pid.getVelocityError();


    pid.calculate(-setPointDegrees);




    output = velocity * SmartDashboard.getNumber("Vel Constant", 0);
    SmartDashboard.putNumber("repeat", SmartDashboard.getNumber("Vel Constant", 0));
    turretPidController.setP(SmartDashboard.getNumber("TurretPID", 0));
    

   SmartDashboard.putNumber("setPoint ROC", velocity);
  if(!locked &&Math.abs(ticksToDegrees(turretMotor.getSelectedSensorPosition())) < 360*3 ){
    setPointTicks = degreesToTicks(setPointDegrees + relativeMark - SmartDashboard.getNumber("yaw", 0));
    pid2.calculate(SmartDashboard.getNumber("yaw", 0));
    SmartDashboard.putNumber("yaw roc", pid2.getVelocityError());
    output += pid2.getVelocityError()*SmartDashboard.getNumber("Yaw Vel Constant", 0);
    output += turretPidController.calculate(turretMotor.getSelectedSensorPosition(), setPointTicks);
    output = srl.calculate(output);
  }else{
     setPointTicks = degreesToTicks(relativeMark);
     output = turretPidControllerAbsolute.calculate(turretMotor.getSelectedSensorPosition(),setPointTicks);

  }

  
    


    
    
    //Calculate and set PID voltage
      
     
      if(output > 10){
        turretMotor.setVoltage(-10);
      }else if(output < -10){
        turretMotor.setVoltage(10);
      }else{
      turretMotor.setVoltage(-output);
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
    //Constantly check if the turret has arrived (used for sequencing), and update the volage to achive the wanted angle
    checkArrival();
    setPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}