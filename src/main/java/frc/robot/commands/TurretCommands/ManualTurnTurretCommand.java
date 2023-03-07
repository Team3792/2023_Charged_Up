// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;
import frc.robot.HelperClasses.*;

public class ManualTurnTurretCommand extends CommandBase {
  /** Creates a new ManualTurnTurretCommand. */
  private TurretSubsystem turretSubsystem;
  Supplier<Double> joystickZRotation, joystickSlider;
  SignalProcessor signalProcessor = new SignalProcessor(12, Constants.TurretConstants.kTurretDeadzone, 0);
  

  public ManualTurnTurretCommand(TurretSubsystem subsystem, Supplier<Double> joystickZRotation, Supplier<Double> joystickSlider) {

    turretSubsystem = subsystem;
    this.joystickSlider = joystickSlider;
    this.joystickZRotation = joystickZRotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.turretMotor.setNeutralMode(NeutralMode.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//Note: It might be benificial to use max angle as the max angle for joystick, so there is a better physical connection for operator
    double rawInput = -joystickZRotation.get();
    
    double processedInput = signalProcessor.getOutput(rawInput);
   // double desiredAngle = processedInput * Constants.TurretConstants.kMaxTurretAngle;
    if(joystickSlider.get() > 0){
     // desiredAngle += 180;
    }

    //turretSubsystem.setPosition(processedInput);
    turretSubsystem.turretMotor.setVoltage(processedInput);
   // turretSubsystem.turretMotor.set(ControlMode.PercentOutput, rawInput);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
