//PURPOSE
//This subsystem needs to encompass all CAN objects on the Turret
//This has one 775 pro motor


//TODO:
//      Get the API library downloaded for this motor 
//      Define motors

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;



public class TurretSubsystem extends SubsystemBase {
  //assumIng talonFX, CORRECT LATER
  TalonFX turretMotor = new TalonFX(Constants.MotorID.kTurretMotor);

  PIDController turretPidController = new PIDController(
    Constants.TurretConstants.kTurretkP, 
    Constants.TurretConstants.kTurretkI, 
    Constants.TurretConstants.kTurretkD);

  public TurretSubsystem() {}

  public void setPosition(double degrees){
    double setPointTicks = degreesToTicks(degrees);

      double output = turretPidController.calculate(turretMotor.getSelectedSensorPosition(), setPointTicks);
      //Should we set PIDs with voltage instead?
      turretMotor.set(ControlMode.PercentOutput, output);
  }

  private double degreesToTicks(double degrees){
    //Assuming 20248 ticks per rev
    double rotationsTurret = degrees / 360;
    double rotationsDriver = rotationsTurret * 462.2;
    double ticks = rotationsDriver * 2048;

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