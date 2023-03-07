// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlipperSubsystem extends SubsystemBase {
  /** Creates a new FlipperSubsystem. */
  WPI_TalonSRX flipperMotor = new WPI_TalonSRX(Constants.MotorID.kFlipperMotor);

  //0 = retracted
  //1 = extended

  public FlipperSubsystem() {}

  public void extend(){
    flipperMotor.setVoltage(Constants.FlipperConstants.kExtendVoltage);
    //update flipper state

  }

  public void retract(){
    flipperMotor.setVoltage(Constants.FlipperConstants.kRetractVoltage);
    //update flipper state after setting voltage

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
