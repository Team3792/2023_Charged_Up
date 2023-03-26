// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.*;
//import frc.robot.commands.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.LEDCommands.LEDShowIntakeStatusCommand;
import frc.robot.commands.Sequences.EngageTurtleMode;
import frc.robot.commands.Sequences.ToElevatorLevel;
import frc.robot.Autonomous.Actions.DropAllAutoCommand;
import frc.robot.Autonomous.Routines.CubeTaxi;
import frc.robot.Autonomous.Routines.TwoConeAutoMantis;
import frc.robot.IntakePreparationCommands.AdjustForConeIntakeCommand;
import frc.robot.IntakePreparationCommands.HighIntakeCubePreparation;
import frc.robot.commands.AutoAimingCommands.AutoAimCommand;
import frc.robot.commands.BoomCommands.ManualExtendBoomCommand;
import frc.robot.commands.DriveCommands.DriveCommand;
import frc.robot.commands.ElevatorCommands.ElevatorMoveAutoCommand;
import frc.robot.commands.ElevatorCommands.ElevatorMoveManualCommand;
import frc.robot.subsystems.*;
import frc.robot.IntakePreparationCommands.LowIntakeCubePreparation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.TurretCommands.ManualTurnTurretCommand;
import frc.robot.commands.TurretCommands.TurretOutOfTurtleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Autonomous.Routines.TwoConeAutoMantis;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //subsystems

  //These three subsystems must be accesable to Robot.java to run the odemetry vision calcultions
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
   public final TurretSubsystem turretSubsystem = new TurretSubsystem();
   //public final VisionSubsystem visionSubsystem = new VisionSubsystem();

 public final BoomSubsystem boomSubsystem = new BoomSubsystem();
   public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
   private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final FlipperSubsystem flipperSubsystem = new FlipperSubsystem();
   private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  



  //controller definitions:
  final Joystick driveJoystick = new Joystick(Constants.ButtonConstant.kDriveJoystick);
  final Joystick operatorJoystick = new Joystick(Constants.ButtonConstant.kOperatorJoystick);

  //Button definitions - should we have these in the constants class?

  //Intake/Extake
  final Trigger coneIntakeButton = new JoystickButton(driveJoystick, Constants.ButtonConstant.kConeIntakeButton);
  final Trigger cubeIntakeButton = new JoystickButton(driveJoystick, Constants.ButtonConstant.kCubeIntakeButton);
  final Trigger dropAllButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kDropAllButton);

  //Elevator

  final Trigger groundElevatorButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kGroundElevatorButton);
  final Trigger middleElevatorButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kMiddleElevatorButton);
  final Trigger highElevatorButton = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kHighElevatorButton);

  //Intake Elevator buttons

  final Trigger cubeIntakeHigh = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kConeIntakeHigh);
  final Trigger cubeIntakeLow = new JoystickButton(operatorJoystick, 3);


  final Trigger engageAutoAim = new JoystickButton(operatorJoystick, 2);

  final Trigger toggleFlipper = new JoystickButton(driveJoystick, Constants.ButtonConstant.kFlipperToggleButton);

  final Trigger engageTurtleMode = new JoystickButton(operatorJoystick, Constants.ButtonConstant.kEngageTurtleModeButton);

  //manual elevator controls
  final Trigger elevatorUp = new POVButton(operatorJoystick, 0);
  final Trigger elevatorDown = new POVButton(operatorJoystick, 180);

  //left and right drop buttons
 
  final Trigger leftLockButton = new POVButton(operatorJoystick, 270);
  final Trigger rightLockButton = new POVButton(operatorJoystick, 90);

  //Programmer mode/zero sensors triggers

  final Trigger programmerModeButton7 = new JoystickButton(operatorJoystick, 7);
  final Trigger programmerModeButton8 = new JoystickButton(operatorJoystick, 8);
  final Trigger programmerModeButton9 = new JoystickButton(operatorJoystick, 9);
  final Trigger programmerModeButton10 = new JoystickButton(operatorJoystick, 10);

  //Variables that include global information of state of robot

   public static String LEDIntakeStatus = "none";

   public static String lastPieceHeld = "none";
  
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
    () -> -driveJoystick.getRawAxis(1), 
    () -> -driveJoystick.getRawAxis(2)));

    // elevatorSubsystem.setDefaultCommand(new ElevatorMoveManualCommand(elevatorSubsystem, 
    // () -> -operatorJoystick.getRawAxis(3)));

    //Manual Aiming bindings (elevator in button bindings)
    turretSubsystem.setDefaultCommand(new ManualTurnTurretCommand(turretSubsystem,
    () -> -operatorJoystick.getRawAxis(2)
    ));

    boomSubsystem.setDefaultCommand(new ManualExtendBoomCommand(boomSubsystem, 
    () -> -operatorJoystick.getRawAxis(1)
    ));

    intakeSubsystem.setDefaultCommand(new StartEndCommand(
    intakeSubsystem::stopIntake, intakeSubsystem::stopIntake, intakeSubsystem));
//By default, retract the flipper
    // flipperSubsystem.setDefaultCommand(
    //   new StartEndCommand(
    //     flipperSubsystem::retract, 
    //     flipperSubsystem::extend, 
    //     flipperSubsystem)
    // );
    

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
    //Intake and drop buttons - Pass in PDH intake currents

   coneIntakeButton.whileTrue(new IntakeConeCommand(
      intakeSubsystem, 
      () -> powerDistribution.getCurrent(Constants.PowerDistributionHubConstants.kPDHIntakeChannel)
    ));

    cubeIntakeButton.whileTrue(new IntakeCubeCommand(
      intakeSubsystem,
      () -> powerDistribution.getCurrent(Constants.PowerDistributionHubConstants.kPDHIntakeChannel)
      ));

   dropAllButton.whileTrue(new DropAllCommand(intakeSubsystem, lastPieceHeld));

    //direction locker buttons

   rightLockButton.onTrue(new InstantCommand(
    turretSubsystem::lockToRight,
    turretSubsystem
   ));

   leftLockButton.onTrue(new InstantCommand(
    turretSubsystem::lockToLeft,
    turretSubsystem
   ));

   //button bindings for programmer mode

   programmerModeButton7.and(programmerModeButton8).and(programmerModeButton9).and(programmerModeButton10).onTrue(
   new InstantCommand(
      this::zeroSensors,
      turretSubsystem,
      elevatorSubsystem,
      boomSubsystem
      )
   );

      //When the toggle flipper button is on true, extend, on false, retract

    // toggleFlipper.toggleOnTrue(
    //   new StartEndCommand(
    //     flipperSubsystem::extend, 
    //     flipperSubsystem::retract, 
    //     flipperSubsystem)
    // );

    // toggleFlipper.toggleOnFalse(
    //   new StartEndCommand(
    //     flipperSubsystem::retract, 
    //     flipperSubsystem::extend, 
    //     flipperSubsystem
    //     )
    // );

    //Elevator buttons

    

     groundElevatorButton.onTrue(new ToElevatorLevel(elevatorSubsystem, turretSubsystem, boomSubsystem, 0, lastPieceHeld));
     middleElevatorButton.whileTrue(new ToElevatorLevel(elevatorSubsystem, turretSubsystem, boomSubsystem, 1, lastPieceHeld));
   //  highElevatorButton.onTrue(new ElevatorMoveAutoCommand(elevatorSubsystem, intakeStatus, 2));

     //Intake Levels

     cubeIntakeHigh.onTrue(new SequentialCommandGroup(
    new TurretOutOfTurtleMode(turretSubsystem),
    new HighIntakeCubePreparation(elevatorSubsystem)

     ));

     cubeIntakeLow.onTrue(new SequentialCommandGroup(
    new TurretOutOfTurtleMode(turretSubsystem),
    new LowIntakeCubePreparation(elevatorSubsystem)

     ));

    coneIntakeButton.onTrue(new AdjustForConeIntakeCommand(elevatorSubsystem));

   engageTurtleMode.onTrue(new EngageTurtleMode(turretSubsystem, elevatorSubsystem, boomSubsystem));

    elevatorUp.whileTrue(
      new StartEndCommand(
      elevatorSubsystem::manualUp, 
      elevatorSubsystem::stop, 
      elevatorSubsystem));

      

      elevatorDown.whileTrue(
        new StartEndCommand(
        elevatorSubsystem::manualDown, 
        elevatorSubsystem::stop, 
        elevatorSubsystem));
   // engageAutoAim.whileTrue(new AutoAimCommand(turretSubsystem, boomSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   // return new SequentialCommandGroup(new Dri);


 return new CubeTaxi(driveSubsystem, turretSubsystem, boomSubsystem, intakeSubsystem, elevatorSubsystem);


  }

  public void zeroSensors (){
    turretSubsystem.turretMotor.setSelectedSensorPosition(0);
    boomSubsystem.boomMotor.setSelectedSensorPosition(0);
    elevatorSubsystem.elevatorMotor.setSelectedSensorPosition(0);
  }

}
