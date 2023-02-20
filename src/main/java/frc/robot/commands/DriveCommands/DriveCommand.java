//PURPOSE OF COMMAND:
//This is the default drive command. Basic PID driving happens here
//Takes Driver Joystick's two axis as double suppliers and outputs the driving paramaters.

//TODO:
//       PID tuning
//       test on actual bot (need to wait for chassis development)


package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.HelperClasses.SignalProcessor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Creating drive subsystem
  private final DriveSubsystem driveSubsystem;

  //Creating gyro
 // private final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.MotorID.kGyro);

  //Defining PID and Feedforward controllers
  private final PIDController leftPIDController = new PIDController(
    Constants.DriveConstants.kDrivekP,
    Constants.DriveConstants.kDrivekI,
    Constants.DriveConstants.kDrivekD);

  private final PIDController rightPIDController = new PIDController(
    Constants.DriveConstants.kDrivekP,
    Constants.DriveConstants.kDrivekI,
    Constants.DriveConstants.kDrivekD);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.DriveConstants.kDriveKS, 
      Constants.DriveConstants.kDriveKV, 
      Constants.DriveConstants.kDriveKA);

  //Defining joysticks
  
  Supplier<Double> joystickForward, joystickRotation;

  //Define signal processors

  SignalProcessor forwardSignalProcessor = new SignalProcessor(Constants.DriveConstants.kMaxDriveSpeed, 0.01, 0);
  SignalProcessor rotationSignalProcessor = new SignalProcessor(Constants.DriveConstants.kMaxDriveSpeed, 0.01, 0);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, Supplier<Double> joystickForward, Supplier<Double> joystickRotation) {
    this.joystickForward = joystickForward;
    this.joystickRotation = joystickRotation;
    driveSubsystem = subsystem;



    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  // Called every time the scheduler runs while the command is scheduled.



  public final void drive(double forwardInput, double rotationInput){


    double forwardOutput = forwardSignalProcessor.getOutput(forwardInput);
    double rotationOutput = forwardSignalProcessor.getOutput(rotationInput);

    var wheelSpeeds = driveSubsystem.differentialDriveKinematics.toWheelSpeeds(new ChassisSpeeds(forwardOutput, 0, rotationOutput));
    setSpeeds(wheelSpeeds);

  } 
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    double leftSpeed = toMeters(driveSubsystem.leftLead.getSelectedSensorVelocity());
    double rightSpeed = toMeters(driveSubsystem.rightLead.getSelectedSensorVelocity());

    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }


  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);

    DifferentialDriveWheelSpeeds currentWheelsSpeeds = getWheelSpeeds();
    double rightOutput = rightPIDController.calculate(
      currentWheelsSpeeds.rightMetersPerSecond, 
      speeds.rightMetersPerSecond) + rightFeedforward;

    double leftOutput = leftPIDController.calculate(
      currentWheelsSpeeds.leftMetersPerSecond, 
      speeds.leftMetersPerSecond) + leftFeedforward;

      driveSubsystem.setVolts(leftOutput, rightOutput);
  }


  public double toMeters(double ticksPerSecond){
    double motorRotationsPerSecond = ticksPerSecond/2048;
    //replace 10 with gear ratio
    double wheelRotationsPerSecond = motorRotationsPerSecond/10;
    double distancePerRotations = Math.PI*Units.inchesToMeters(6);
    double metersPerSecond = distancePerRotations * wheelRotationsPerSecond;
    return metersPerSecond;
  }
 
  @Override

  public void execute() {
    drive(joystickForward.get(), joystickRotation.get());

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
