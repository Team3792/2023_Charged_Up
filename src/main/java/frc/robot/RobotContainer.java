// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.function.Supplier;

// import frc.robot.Constants.*;
//import frc.robot.commands.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.LEDCommands.LEDShowIntakeStatusCommand;
import frc.robot.IntakePreparationCommands.AdjustForCubeIntakeCommand;
import frc.robot.IntakePreparationCommands.HighIntakeConePreparation;
import frc.robot.commands.BoomCommands.ManualExtendBoomCommand;
import frc.robot.commands.DriveCommands.DriveCommand;
import frc.robot.commands.ElevatorCommands.ElevatorMoveAutoCommand;
import frc.robot.subsystems.*;
import frc.robot.IntakePreparationCommands.LowIntakeConePreparation;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TurretCommands.ManualTurnTurretCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final BoomSubsystem boomSubsystem = new BoomSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();


  //controller definitions:
  final Joystick driveJoystick = new Joystick(Constants.ButtonConstant.kDriveJoystick);
  final Joystick operatorJoystick = new Joystick(Constants.ButtonConstant.kOperatorJoystick);

  //Button definitions - should we have these in the constants class?

  //Intake/Extake
  final Trigger coneIntakeButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kConeIntakeButton);
  final Trigger cubeIntakeButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kCubeIntakeButton);
  final Trigger dropAllButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kDropAllButton);

  //Elevator

  final Trigger groundElevatorButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kGroundElevatorButton);
  final Trigger middleElevatorButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kMiddleElevatorButton);
  final Trigger highElevatorButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kHighElevatorButton);

  //Intake Elevator buttons

  final Trigger coneIntakeHigh = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kConeIntakeHigh);
  final Trigger coneIntakeLow = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kConeIntakeLow);





  //Variables that include global information of state of robot

   public static String intakeStatus = "none";
  
  //"none" "cube" "cone" "intaking cube" "intaking cone" "dropping"

   public static String lastIntakeHeight = "none";
   //Will be "none" "high" or "low"

   public static int elevatorHeight = 0;
  /* 0 = ground
   * 1 = mid
   * 2 = high
   */



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, 
    () -> driveJoystick.getRawAxis(1), 
    () -> driveJoystick.getRawAxis(2)));

    //Manual Aiming bindings (elevator in button bindings)
    turretSubsystem.setDefaultCommand(new ManualTurnTurretCommand(turretSubsystem,
    () -> operatorJoystick.getRawAxis(0),
    () -> operatorJoystick.getRawAxis(3)
    ));

    boomSubsystem.setDefaultCommand(new ManualExtendBoomCommand(boomSubsystem, 
    () -> operatorJoystick.getRawAxis(1)
    ));

    

    //Setting up LED system where the lights change depending on intake status

    ledSubsystem.setDefaultCommand(new LEDShowIntakeStatusCommand(ledSubsystem));

    
    
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    //Intake and drop buttons
    coneIntakeButton.whileTrue(new IntakeConeCommand(intakeSubsystem));
    cubeIntakeButton.whileTrue(new IntakeCubeCommand(intakeSubsystem));
    dropAllButton.whileTrue(new DropAllCommand(intakeSubsystem, intakeStatus));

    //Elevator buttons

    groundElevatorButton.onTrue(new ElevatorMoveAutoCommand(elevatorSubsystem, intakeStatus, 0));
    middleElevatorButton.onTrue(new ElevatorMoveAutoCommand(elevatorSubsystem, intakeStatus, 1));
    highElevatorButton.onTrue(new ElevatorMoveAutoCommand(elevatorSubsystem, intakeStatus, 2));

    //Intake Levels

    coneIntakeHigh.onTrue(new HighIntakeConePreparation(elevatorSubsystem));
    coneIntakeLow.onTrue(new LowIntakeConePreparation(elevatorSubsystem));
    cubeIntakeButton.onTrue(new AdjustForCubeIntakeCommand(elevatorSubsystem));

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
}
