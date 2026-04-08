package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);
  
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );
  
  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;
  
  final public IntakeSubsystem intake = new IntakeSubsystem();
  final ShooterSubsystem shooter = new ShooterSubsystem();
  final HopperSubsystem hopper = new HopperSubsystem();
  final ClimberSubsystem climber = new ClimberSubsystem();
  final LimelightSubsystem limelight = new LimelightSubsystem(this);

  public enum States {
    FORWARD(1.0),
    BACKWARDS(-1.0),
    NONE(0.0);

    private final double multiplier;

    States(double multiplier) {
        this.multiplier = multiplier;
      }

      public double getMultiplier() { 
        return multiplier;
    }
  }
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driverController.getLeftY() * -1,
  () -> driverController.getLeftX() * -1)
  .withControllerRotationAxis(driverController::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(1)
  .scaleRotation(0.85)
  .allianceRelativeControl(true);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
  */
 public RobotContainer()
 {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    // NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    // NamedCommands.registerCommand("Shoot", Commands.parallel(shooter.runShooter(21).repeatedly(), hopper.runHopper(States.FORWARD, shooter::isShooterAtSpeed)).withTimeout(5));
    // NamedCommands.registerCommand("Shoot2", Commands.parallel(shooter.runShooter(21).repeatedly(), hopper.runHopper(States.FORWARD, shooter::isShooterAtSpeed)).withTimeout(10));
    // NamedCommands.registerCommand("zeroGryo", drivebase.zeroGyroCommand());

    NamedCommands.registerCommand("Extend_Intake", intake.intakeDown);
    NamedCommands.registerCommand("Intake", intake.runIntake(States.FORWARD, 10).repeatedly());
    NamedCommands.registerCommand("Auto_Aim_Shoot",
    new AutoAlign(
      drivebase,
      limelight,
      shooter,
      hopper,
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1
    ).repeatedly());
    
    NamedCommands.registerCommand("Agitate_Fuel", Commands.sequence(
      intake.runIntake(States.BACKWARDS, 3 ).withTimeout(0.1),
      intake.runIntake(States.FORWARD, 12).withTimeout(0.5),
      Commands.waitSeconds(0.5)
    ).repeatedly());
    
    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();
    
    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    
    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
  */
 private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    BooleanSupplier trueSupplier = () -> true;
    
    //=========================================================================
    //                                driver
    //=========================================================================
    
    // Reset Gyro -> Start Button
    driverController.start()
    .onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    
    // Drive Slow -> Right Trigger
    driverController.rightTrigger()
    .onTrue(Commands.runOnce(() -> driveAngularVelocity
    .scaleTranslation(0.5).scaleRotation(0.3)))
    .onFalse(Commands.runOnce(() -> driveAngularVelocity.scaleTranslation(1.0).scaleRotation(0.85)));

    // Auto-Aim Shoot -> X
    driverController.x().whileTrue(
      new AutoAlign(
        drivebase,
        limelight,
        shooter,
        hopper,
        () -> driverController.getLeftY() * -1,
        () -> driverController.getLeftX() * -1
      )
    );

    // Shoot Low -> A
    driverController.a().whileTrue(
      Commands.parallel(
        hopper.runHopper(States.FORWARD, shooter::isShooterAtSpeed),
        shooter.runShooter(22
        )
      )
    );
      
      // Shoot Medium -> B
      driverController.b().whileTrue(
        Commands.parallel(
          hopper.runHopper(States.FORWARD, shooter::isShooterAtSpeed),
          shooter.runShooter(25)
        )
      );
      
      // Shoot High -> Y
      driverController.y().whileTrue(
        Commands.parallel(
          hopper.runHopper(States.FORWARD, shooter::isShooterAtSpeed),
          shooter.runShooter(28)
        )
      );




      //=========================================================================
      //                               operator
      //=========================================================================

      // Intake -> Right Trigger
      operatorController.rightTrigger().whileTrue(
        intake.runIntake(States.FORWARD, 10)
      );
      
      // Dump Fuel -> Left Trigger
      operatorController.leftTrigger().whileTrue(
        Commands.parallel(
          hopper.runHopper(States.BACKWARDS, trueSupplier),
          intake.runIntake(States.BACKWARDS, 9)
        )
      );
      
      // Extend Intake -> Right Bumper
      operatorController.rightBumper().onTrue(
        intake.intakeDown
      );
      
      // Agitate Fuel -> Left Bumper
      operatorController.leftBumper().onTrue(
        Commands.sequence(
          intake.runIntake(States.BACKWARDS, 3).withTimeout(0.1),
          intake.runIntake(States.FORWARD, 12).withTimeout(0.5)
        )
      );

      // Hopper Floor -> A
      operatorController.a().whileTrue(
        hopper.runHopper(States.BACKWARDS, trueSupplier)
      );

      // Intake Up -> X
      operatorController.x().onTrue(
          intake.intakeUp
      );

      // // Climber Up -> D-Pad Up
      // operatorController.povUp().whileTrue(
      //   climber.runClimber(States.FORWARD)
      // );
      
      // // Climber Down -> D-Pad Down
      // operatorController.povDown().whileTrue(
      //   climber.runClimber(States.BACKWARDS)
      // );
      
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // return Commands.parallel(shooter.runShooter(20.5).repeatedly(), hopper.runHopper(States.FORWARD, shooter::isShooterAtSpeed)).withTimeout(8);
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }
  
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}