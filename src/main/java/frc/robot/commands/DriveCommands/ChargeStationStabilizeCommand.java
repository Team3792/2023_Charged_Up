// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.io.Console;

import org.apache.commons.collections4.map.PassiveExpiringMap.ConstantTimeToLiveExpirationPolicy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class ChargeStationStabilizeCommand extends CommandBase {
  /** Creates a new ChargeStationStabilizeCommand. */
  DriveSubsystem driveSubsystem;

  //Creates a PID controller that will use the pitch to output a forward component for the arcade drive method
  PIDController pidController = new PIDController(
    Constants.ChargeStationStabilizeConstants.kP,
    Constants.ChargeStationStabilizeConstants.kI,
    Constants.ChargeStationStabilizeConstants.kD);

  public ChargeStationStabilizeCommand(DriveSubsystem driveSubsystem) {

    //define and require the drivesubsystem
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get pitch from driveSubsystem's pigeon
    double pitch = driveSubsystem.getPitch();

    //Calculate the next term fromt the PID
    //The error = 0-pitch = -pitch, since the setpoint is always 0 degrees (ie. level)
    double outputVoltage = pidController.calculate(-pitch);

//If pitch is in deadband, stop and break 
if(Math.abs(pitch) < Constants.ChargeStationStabilizeConstants.kPitchDeadband){
  driveSubsystem.stopAndBreak();
}
//Check direction trying to move, if voltage is not enough for that direction, set it to the minimum voltage
else if(outputVoltage > 0 && outputVoltage < Constants.ChargeStationStabilizeConstants.kMinVoltage){
      outputVoltage = Constants.ChargeStationStabilizeConstants.kMinVoltage;
}else if(outputVoltage < 0 && outputVoltage > -Constants.ChargeStationStabilizeConstants.kMinVoltage){
      outputVoltage = -Constants.ChargeStationStabilizeConstants.kMinVoltage;
    }

    //Put this voltage on the motors
    driveSubsystem.setVoltage(outputVoltage, outputVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopAndBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
