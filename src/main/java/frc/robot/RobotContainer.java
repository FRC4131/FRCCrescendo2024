// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.Swerve.TRACK_WIDTH;
import static frc.robot.Constants.Swerve.WHEEL_BASE;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_drivetrainSubsystem);
  
  // Xbox Controllers (Replace with CommandPS4Controller or CommandJoystick if needed)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureAutoBuilder(); // Configure PathPlanner AutonBuilder 
    setDefaultCommands();  // Set/Bind the default commands for subsystems (i.e. commands that will run if the SS isn't actively running a command)
    configureBindings();  // Configure any game controller bindings and Triggers
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
  }

  public void setDefaultCommands() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driverController.getLeftX(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis(),
        true));
  }

  private void configureBindings() {
    // Schedule Triggers    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
