// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GoToNoteCommand;
import frc.robot.commands.GoToPoseTeleopCommand;
import frc.robot.commands.StdDevEstimatorCommand;
import frc.robot.commands.TargetAmpCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.Swerve.TRACK_WIDTH;
import static frc.robot.Constants.Swerve.WHEEL_BASE;

import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems:
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); 
  private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_drivetrainSubsystem, m_visionSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); 
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(); 
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(); 

  //set to color specific constants later on 
  private Pose2d m_speakerPose; 
  private Pose2d m_ampPose; 
  private Pose2d m_sourcePose; 
  // private Pose2d m_stagePose; 
  private double m_directionInvert; 
  private double m_angleOffset; 

  private SendableChooser<Command> m_autoChooser; //for autons
  
  // Xbox Controllers (Replace with CommandPS4Controller or CommandJoystick if needed)
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setAllianceSpecific(); //sets constants to be either blue or red 
    configureAutoBuilder(); // Configure PathPlanner AutonBuilder 
    setDefaultCommands();  // Set/Bind the default commands for subsystems (i.e. commands that will run if the SS isn't actively running a command)
    configureDriverBindings();  // Configure driver game controller bindings and Triggers
    configureOperatorBindings();  //Configure operator game controller bindings and Triggers
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
   
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value, boolean squareAxis) {
    // Deadband
    value = deadband(value, 0.075);

    // Square the Axis
    if (squareAxis) {
      value = Math.copySign(value * value, value);
    }
    return value;
  }

  public void setAllianceSpecific()
  {

    Optional<Alliance> alliance = DriverStation.getAlliance(); 
    
    SmartDashboard.putString("Alliance", "blue");
    m_speakerPose = Constants.FieldConstants.BLUE_SPEAKER;
    m_sourcePose = Constants.FieldConstants.BLUE_SOURCE_RIGHT;
    m_ampPose = Constants.FieldConstants.BLUE_AMP;
    m_directionInvert = 1.0;
    m_angleOffset = 0.0;

    if (!alliance.isEmpty()) {
      if (alliance.get().equals(Alliance.Red)) {
        SmartDashboard.putString("Alliance", "red");
        m_speakerPose = Constants.FieldConstants.RED_SPEAKER;
        m_sourcePose = Constants.FieldConstants.RED_SOURCE_RIGHT;
        m_ampPose = Constants.FieldConstants.RED_AMP;
        m_directionInvert = -1.0;
        m_angleOffset = Math.PI;
      }
    }
  }

  public void configureAutoBuilder() {
    
    //Configure the autobuilder from pathplanner (Holonomic for Swerve drive)
    AutoBuilder.configureHolonomic(
        m_poseEstimationSubsystem::getPose, // Robot pose supplier
        m_poseEstimationSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        m_drivetrainSubsystem::getChassisSpeed,// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_drivetrainSubsystem::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(3.5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(6.0, 0.0, 0.0), // Rotation PID constants
            MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
            Math.sqrt(TRACK_WIDTH*TRACK_WIDTH + WHEEL_BASE*WHEEL_BASE) / 2.0, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_drivetrainSubsystem // Reference to this subsystem to set requirements
    );

    //Named Commands 
    NamedCommands.registerCommand("Intake On", m_intakeSubsystem.setPowerCommand(-0.7));
    NamedCommands.registerCommand("Intake Off", m_intakeSubsystem.setPowerCommand(0.0));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public void setDefaultCommands() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        () -> m_directionInvert * -modifyAxis(m_driverController.getLeftY(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis(),
        true));

  }

  private void configureDriverBindings() {
    // Schedule Triggers 
    m_driverController.back().onTrue(m_poseEstimationSubsystem.zeroAngleCommand(m_angleOffset)); 
    m_driverController.a().whileTrue(new GoToPoseTeleopCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, 0,  
    () -> m_directionInvert * -modifyAxis(m_driverController.getLeftY(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
         () -> m_driverController.getLeftTriggerAxis(),
         true,
          m_speakerPose));

    m_driverController.povLeft().whileTrue(new GoToNoteCommand(m_drivetrainSubsystem,
       m_visionSubsystem,
      m_intakeSubsystem, 
      () -> m_driverController.getLeftTriggerAxis(),
      false)); 

    m_driverController.b().whileTrue(new TargetAmpCommand(m_drivetrainSubsystem,
       m_poseEstimationSubsystem,
      () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,   
      () -> m_driverController.getLeftTriggerAxis(), 
      true, 
       m_ampPose)); 
      

    //Only allow intake when the feeder state shows it's ready for more input AND the driver presses the button
    // Trigger intakeTrigger = new Trigger(m_feederSubsystem::intakeAllowed);
    // m_driverController.x().and(intakeTrigger)
        // .onTrue(m_intakeSubsystem.setPowerCommand(-0.7)).onFalse(m_intakeSubsystem.setPowerCommand(0.0));

    m_driverController.x().onTrue(m_intakeSubsystem.setPowerCommand(-0.7)).onFalse(m_intakeSubsystem.setPowerCommand(0));
    m_driverController.y().onTrue(m_intakeSubsystem.setPowerCommand(0.7)).onFalse(m_intakeSubsystem.setPowerCommand(0));
    m_driverController.rightBumper().onTrue(m_feederSubsystem.setFeederPowerCommand(-0.7)).onFalse(m_feederSubsystem.setFeederPowerCommand(0));//feeder runs as intake runs (testing without beam breaks rn)
    m_driverController.leftBumper().onTrue(m_feederSubsystem.setFeederPowerCommand(0.7)).onFalse(m_feederSubsystem.setFeederPowerCommand(0));

    new Trigger (() -> m_visionSubsystem.seesTargets()).whileTrue(new StdDevEstimatorCommand(m_visionSubsystem));

    //automatic spatial trigger
    // new Trigger (() -> m_poseEstimationSubsystem.isInRadius(new Pose2d(m_speakerPose.getX(), //blue speaker
    //     m_speakerPose.getY(),
    //     new Rotation2d()), 1.5))
    //       .whileTrue(new GoToPoseTeleopCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, 0,  
    //           () -> -modifyAxis(m_driverController.getLeftY(), false) *
    //               MAX_VELOCITY_METERS_PER_SECOND,
    //           () -> -modifyAxis(m_driverController.getLeftX(), false) *
    //               MAX_VELOCITY_METERS_PER_SECOND,
    //           () -> m_driverController.getLeftTriggerAxis(),
    //           true,
    //           m_speakerPose));

  }

  public void configureOperatorBindings()
  {
    m_operatorController.rightBumper().whileTrue(
      m_armSubsystem.manualArmCommand(0.2)).onFalse(m_armSubsystem.manualArmCommand(0));

        m_operatorController.leftBumper().whileTrue(
      m_armSubsystem.manualArmCommand(-0.2)).onFalse(m_armSubsystem.manualArmCommand(0));

      m_operatorController.x().onTrue(m_armSubsystem.rotateToAngleCommand(25));
      m_operatorController.a().onTrue(m_armSubsystem.rotateToAngleCommand(90.0));
      m_operatorController.back().onTrue(m_armSubsystem.resetArmEncoderCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_autoChooser.getSelected();
    // return new PathPlannerAuto("testing"); 
  }
}
