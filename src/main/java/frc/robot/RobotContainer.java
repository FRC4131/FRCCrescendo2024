// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.ArmJoystickCommand;
import frc.robot.commands.AutoArmCommand;
import frc.robot.commands.AutonGoToNoteCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonShootCommand;
import frc.robot.commands.AutonGoToPoseWithArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GoToNoteCommand;
//import frc.robot.commands.GoToNoteCommand;
import frc.robot.commands.GoToPoseTeleopCommand;
import frc.robot.commands.GoToPoseWithArmCommand;
import frc.robot.commands.StdDevEstimatorCommand;
import frc.robot.commands.TargetAmpCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.Swerve.TRACK_WIDTH;
import static frc.robot.Constants.Swerve.WHEEL_BASE;

import java.sql.Driver;
import java.util.Optional;
import java.util.concurrent.locks.Condition;

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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_intakeSubsystem); 
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(); 
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(); 
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(); 

  //set to color specific constants later on 
  private Pose2d m_speakerPose; 
  private Pose2d m_ampPose; 
  //private Pose2d m_sourcePose; 
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
    setAllianceSpecific();
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


  public void initializeTeleop()
  {
    m_armSubsystem.resetOffset();
    m_intakeSubsystem.setPower(0.0);
    m_feederSubsystem.setPower(0.0); 
    m_shooterSubsystem.setPower(0.0);
    m_climberSubsystem.setPower(0.0);
    m_drivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
    //m_armSubsystem.goToAngle(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
    //m_armSubsystem.resetEncoderCommand(); 
  }

  public void initializeAuton()
  {
    m_armSubsystem.setEncodertoPropAngle(); 
  }

  public void setAllianceSpecific()
  {
    DriverStation.refreshData();
    Optional<Alliance> alliance = DriverStation.getAlliance(); 

    SmartDashboard.putString("Alliance", "blue");
    m_speakerPose = Constants.FieldConstants.BLUE_SPEAKER;
    //m_sourcePose = Constants.FieldConstants.BLUE_SOURCE_RIGHT;
    m_ampPose = Constants.FieldConstants.BLUE_AMP;
    m_directionInvert = 1.0;
    m_angleOffset = 0.0;

    if (!alliance.isEmpty()) {
      if (alliance.get().equals(Alliance.Red)) {
        SmartDashboard.putString("Alliance", "red");
        m_speakerPose = Constants.FieldConstants.RED_SPEAKER;
        //m_sourcePose = Constants.FieldConstants.RED_SOURCE_RIGHT;
        m_ampPose = Constants.FieldConstants.RED_AMP;
        m_directionInvert = -1.0;
        m_angleOffset = Math.PI;
      }
    }
  }

  public void armToRest()
  {
     m_armSubsystem.goToAngle(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE); 
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

    
  //  NamedCommands.registerCommand("Intake", new AutonIntakeCommand(m_intakeSubsystem, m_feederSubsystem));
  NamedCommands.registerCommand("Intake", 
    m_feederSubsystem.setFeederPowerCommand(0.5).alongWith(m_intakeSubsystem.setPowerCommand(1.0))
   .andThen(new WaitUntilCommand(()-> !m_feederSubsystem.getShooterBreaker()))
    // .andThen(new WaitCommand(3))
    .andThen(m_feederSubsystem.setFeederPowerCommand(0.0).alongWith(m_intakeSubsystem.setPowerCommand(0.0)))
    // .andThen(m_shooterSubsystem.setPowerCommand(0.5).andThen(new WaitCommand(3)).andThen(m_shooterSubsystem.setPowerCommand(0.0)))
    ); 
    // NamedCommands.registerCommand("Shoot Speaker", m_shooterSubsystem.setPowerCommand(1.0).andThen(new WaitCommand(0.5))
    //  .andThen(m_feederSubsystem.setFeederPowerCommand(1)).andThen(new WaitCommand(1.0)));
    // NamedCommands.registerCommand("Stop Shooter", m_shooterSubsystem.setPowerCommand(0.0).andThen(m_feederSubsystem.setFeederPowerCommand(0.0)));
   // NamedCommands.registerCommand("Shoot Amp", new AutonShootCommand(m_feederSubsystem, m_shooterSubsystem, 0.7));
   NamedCommands.registerCommand("Arm Rest Angle", new AutoArmCommand(m_armSubsystem, Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE));
    NamedCommands.registerCommand("Set Arm Angle Prop", m_armSubsystem.setEncodertoPropAngle());
    NamedCommands.registerCommand("Arm off prop", new AutoArmCommand(m_armSubsystem, Constants.ArmConstants.ARM_OFF_PROP).andThen(new WaitCommand(0.5)));
    NamedCommands.registerCommand("Go To Note", new AutonGoToNoteCommand(m_drivetrainSubsystem, m_visionSubsystem, m_intakeSubsystem).withTimeout(1.5));
    NamedCommands.registerCommand("Slow Shoot (No Wait)", (m_feederSubsystem.setFeederPowerCommand(1))
      .andThen(new WaitCommand(1.0))
     .andThen(m_feederSubsystem.setFeederPowerCommand(0.0)));
    NamedCommands.registerCommand("Intake On", m_intakeSubsystem.setPowerCommand(1.0));

    NamedCommands.registerCommand("Slow spin up", m_shooterSubsystem.setPowerCommand(0.1));
    NamedCommands.registerCommand("Spin up Shooter", m_shooterSubsystem.setPowerCommand(1.0));
    NamedCommands.registerCommand("Shoot (No Wait)", new AutonGoToPoseWithArmCommand(m_drivetrainSubsystem, m_armSubsystem, 
    m_poseEstimationSubsystem, 0, ()-> 0.0, ()-> 0.0, ()-> 0.0 , true, () -> m_speakerPose).withTimeout(1.5)
    .andThen(new WaitUntilCommand(() -> m_shooterSubsystem.isSpunUp()))
     .andThen(m_feederSubsystem.setFeederPowerCommand(1)).andThen(new WaitCommand(0.5))
     .andThen(m_feederSubsystem.setFeederPowerCommand(0.0)).andThen(new AutoArmCommand(m_armSubsystem, Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE)));
    
    NamedCommands.registerCommand("Shoot (No Align)", (m_feederSubsystem.setFeederPowerCommand(1)).andThen(new WaitCommand(0.5))
     .andThen(m_feederSubsystem.setFeederPowerCommand(0.0)).andThen(new AutoArmCommand(m_armSubsystem, Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE)));

    NamedCommands.registerCommand("Shoot", m_shooterSubsystem.setPowerCommand(1.0).andThen(new AutonGoToPoseWithArmCommand(m_drivetrainSubsystem, m_armSubsystem
    , m_poseEstimationSubsystem, 0, ()-> 0.0, ()-> 0.0, ()-> 0.0 , true, () -> m_speakerPose).withTimeout(1.5))
     .andThen(m_feederSubsystem.setFeederPowerCommand(1)).andThen(new WaitCommand(0.5)).andThen(m_shooterSubsystem.setPowerCommand(0.0))
     .andThen(m_feederSubsystem.setFeederPowerCommand(0.0)).andThen(new AutoArmCommand(m_armSubsystem, Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE)));

    NamedCommands.registerCommand("Shooter off", m_shooterSubsystem.setPowerCommand(0.0));

    NamedCommands.registerCommand("Offset up", m_armSubsystem.setOffsetCommand(0.254));
    NamedCommands.registerCommand("Offset down", m_armSubsystem.setOffsetCommand(-0.254));
    NamedCommands.registerCommand("Reset Offset", m_armSubsystem.hardSetOffsetCommand(0.0));

    NamedCommands.registerCommand("Auton End", new InstantCommand (() -> {
        DataLogManager.log("AUTON END");
    }
        )); 

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    
  }

  public void setDefaultCommands() {

    //drive w joysticks + boost right trigger 
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        () -> m_directionInvert * -modifyAxis(m_driverController.getLeftY(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
            MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getRightTriggerAxis(),
        true));

  }

  private void configureDriverBindings() {
    // Schedule Triggers 

      // m_driverController.y().onTrue(m_shooterSubsystem.setPowerCommand(0.77).andThen(new WaitCommand(0.5)).andThen(m_feederSubsystem.setFeederPowerCommand(1.0)))
      //   .onFalse(m_shooterSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 
        // y -- outtake 
    m_driverController.y().onTrue(m_intakeSubsystem.setPowerCommand(-0.7).alongWith(m_feederSubsystem.setFeederPowerCommand(-0.7))
      ).onFalse((m_intakeSubsystem.setPowerCommand(0.0)).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 

    //back -- reset heading 
    m_driverController.back().onTrue(m_poseEstimationSubsystem.zeroAngleCommand(() -> m_angleOffset)); 

    // //x -- go to resting 
    // m_driverController.x().onTrue(m_armSubsystem.rotateToAngleCommand(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE)); 

    // //left trigger -- go to pose speaker w arm movement 
    // m_driverController.leftTrigger().whileTrue(new GoToPoseWithArmCommand(m_drivetrainSubsystem, m_armSubsystem,
    //   m_poseEstimationSubsystem, 0,  
    //     () -> m_directionInvert * -modifyAxis(m_driverController.getLeftY(), false) *
    //         MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
    //         MAX_VELOCITY_METERS_PER_SECOND,
    //      () -> m_driverController.getRightTriggerAxis(),
    //      true, () -> m_speakerPose));

    // //a -- shoot with the wait 
    m_driverController.b().onTrue(m_shooterSubsystem.setPowerCommand(0.4).andThen(new WaitCommand(0.25)).andThen(m_feederSubsystem.setFeederPowerCommand(1.0)))
        .onFalse(m_shooterSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 

    m_driverController.leftTrigger().onTrue(m_shooterSubsystem.setPowerCommand(0.1).andThen(new WaitCommand(0.25)).andThen(m_feederSubsystem.setFeederPowerCommand(1.0)))
        .onFalse(m_shooterSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 

  // b intake 
      m_driverController.a().and(new Trigger(()-> m_feederSubsystem.getShooterBreaker()))
      .whileTrue(m_intakeSubsystem.setPowerCommand(1.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.5)))
      .onFalse(m_intakeSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0)));

    //b -- amp shoot
    // m_driverController.b().onTrue(m_shooterSubsystem.setPowerCommand(0.15).alongWith(m_feederSubsystem.setFeederPowerCommand(1.0)))
    //   .onFalse(m_shooterSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 



    // //left bumper -- amp align 
    // m_driverController.leftBumper().whileTrue(new TargetAmpCommand(m_drivetrainSubsystem,
    //    m_poseEstimationSubsystem,
    //    m_armSubsystem, 
    //    m_visionSubsystem,
    //   () -> m_directionInvert * -modifyAxis(m_driverController.getLeftY(), false) *
    //         MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
    //         MAX_VELOCITY_METERS_PER_SECOND,   
    //   () -> m_driverController.getRightTriggerAxis(), 
    //   true, () -> m_ampPose)); 

    // // //right bumper -- go to note 
    // m_driverController.rightBumper().whileTrue(new GoToNoteCommand(m_drivetrainSubsystem, 
    //   m_visionSubsystem,
    //    m_intakeSubsystem, 
    //   () -> m_directionInvert * -modifyAxis(m_driverController.getLeftY(), false) *
    //         MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> m_directionInvert * -modifyAxis(m_driverController.getLeftX(), false) *
    //         MAX_VELOCITY_METERS_PER_SECOND,   
    //   () -> m_driverController.getRightTriggerAxis(),
    //    false)); 

    //right arrow -- arm go to back stop
    m_driverController.povRight().onTrue(m_armSubsystem.rotateToAngleCommand(Constants.ArmConstants.ARM_STRAIGHT_UP));

    //left arrow -- arm go to front stop 
    m_driverController.povLeft().onTrue(m_armSubsystem.rotateToAngleCommand(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE));
    
    //up arrow -- manual move arm up 
    m_driverController.povUp().whileTrue(m_armSubsystem.manualModeCommand(0.08)); 

    //down arrow -- manual move arm down 
    m_driverController.povDown().whileTrue(m_armSubsystem.manualModeCommand(-0.08)); 

   // new Trigger(()-> m_feederSubsystem.getShooterBreaker())
   //    .onFalse(m_intakeSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0)));

    // m_driverController.b().and(new Trigger(()-> m_feederSubsystem.getShooterBreaker()))
    // .whileTrue(m_intakeSubsystem.setPowerCommand(0.7).alongWith(m_feederSubsystem.setFeederPowerCommand(0.5)))
    // .onFalse(m_intakeSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0)));

    
    // m_driverController.rightTrigger().whileTrue(m_intakeSubsystem.setPowerCommand(0.7).alongWith(m_feederSubsystem.setFeederPowerCommand(0.7))
      // ).onFalse((m_intakeSubsystem.setPowerCommand(0.0)).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 

    //y -- outake
    // m_driverController.y().whileTrue(m_intakeSubsystem.setPowerCommand(-0.7).alongWith(m_feederSubsystem.setFeederPowerCommand(-0.7))
    //   ).onFalse((m_intakeSubsystem.setPowerCommand(0.0)).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0)));

    //pov up/down -- adjusts offset for the auto speaker align by 1 INCH


    // //rumble for shooter ready 
    // new Trigger(() -> m_shooterSubsystem.isSpunUp())
    //   .whileTrue(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.2); 
    //   })).onFalse(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0); 
    //   }));

    // new Trigger(() -> m_visionSubsystem.seesSpeakerTags().and(()-> Dri)
    //         .whileTrue(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.2); 
    //   })).onFalse(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.0); 
    //   }));

    // new Trigger(() -> m_visionSubsystem.seesAmpTags())
    //         .whileTrue(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.2); 
    //   })).onFalse(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0); 
    //   }));


    //trigger for april tag updating 
    // new Trigger(() -> m_poseEstimationSubsystem.isInRadius(m_speakerPose, 3.0)
    //   .and(m_poseEstimationSubsystem.aprilTagUpdating()))
    //   .whileTrue(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.2); 
    //   })).onFalse(new InstantCommand (() -> {
    //     m_driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.0); 
    //   }));

    //trigger for sees note
    new Trigger(() -> m_visionSubsystem.seesNote())
      .whileTrue(new InstantCommand (() -> {
        m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.2); 
      })).onFalse(new InstantCommand (() -> {
        m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0); 
      }));

    


    //Only allow intake when the feeder state shows it's ready for more input AND the driver presses the button
    // Trigger intakeTrigger = new Trigger(m_feederSubsystem::intakeAllowed);
    // m_driverController.x().and(intakeTrigger)
        // .onTrue(m_intakeSubsystem.setPowerCommand(-0.7)).onFalse(m_intakeSubsystem.setPowerCommand(0.0));

    //new Trigger (() -> m_visionSubsystem.seesTargets()).whileTrue(new StdDevEstimatorCommand(m_visionSubsystem));

    //automatic spatial trigger
    // new Trigger (() -> m_poseEstimationSubsystem.isInRadius(new Pose2d(m_speakerPose.getX(), //blue speaker
    //     m_speakerPose.getY(),
    //     new Rotation2d()), 1.5))
    //       .whileTrue(new GoToPoseTeleopCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, 0,  
    //           () -> -modifyAxis(m_driverController.getLeftY(), false) *
    //               MAX_VELOCITY_METERS_PER_SECOND,
    //           () -> -modifyAxis(m_driverController.getLeftX(), false) *
    //               MAX_VELOCITY_METERS_PER_SECOND,
    //           () -> m_driverController.getRightTriggerAxis(),
    //           true,
    //           m_speakerPose));

    // new Trigger(() -> m_poseEstimationSubsystem.isInRadius(m_speakerPose, 1.0))
    //   .and(new Trigger(() -> !m_feederSubsystem.getShooterBreaker())).whileTrue(m_shooterSubsystem.setPowerCommand(1.0))
    //   .alongWith(new InstantCommand(() -> {
    //     m_shooterSpedUp = true; 
    //   }))
    //   .onFalse(m_shooterSubsystem.setPowerCommand(0.0)); 

    //new Trigger(() -> m_armSubsystem.frontLimitSwitch()).onTrue(m_armSubsystem.resetEncoderCommand()); 

  }

  public void configureOperatorBindings()
  {
    //right arrow -- arm go to back stop
    m_operatorController.povRight().onTrue(m_armSubsystem.rotateToAngleCommand(Constants.ArmConstants.ARM_STRAIGHT_UP));

    //left arrow -- arm go to front stop 
    m_operatorController.povLeft().onTrue(m_armSubsystem.rotateToAngleCommand(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE));

    //back -- reset encoder
    m_operatorController.back().onTrue(m_armSubsystem.resetEncoderCommand());

    // right trigger -- climb
    m_operatorController.rightTrigger().onTrue(m_climberSubsystem.setPowerCommand(0.7)).onFalse(m_climberSubsystem.setPowerCommand(0.0)); 

    //right bumper -- undo climb 
    m_operatorController.rightBumper().onTrue(m_climberSubsystem.setPowerCommand(-0.7)).onFalse(m_climberSubsystem.setPowerCommand(0.0)); 
    //m_operatorController.b().and(m_operatorController.rightBumper()).onTrue(m_climberSubsystem.setPowerCommand(-1.0)).onFalse(m_climberSubsystem.setPowerCommand(0.0)); 

    //b -- intake
    m_operatorController.b().and(new Trigger(()-> m_feederSubsystem.getShooterBreaker()))
      .whileTrue(m_intakeSubsystem.setPowerCommand(1.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.5)))
      .onFalse(m_intakeSubsystem.setPowerCommand(0.0).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0)));

    //a -- shoot no wait 
    m_operatorController.a().whileTrue(m_shooterSubsystem.setPowerCommand(1.0)).onFalse(m_shooterSubsystem.setPowerCommand(0.0)); 

    // y -- outtake 
    m_operatorController.y().onTrue(m_intakeSubsystem.setPowerCommand(-0.7).alongWith(m_feederSubsystem.setFeederPowerCommand(-0.7))
      ).onFalse((m_intakeSubsystem.setPowerCommand(0.0)).alongWith(m_feederSubsystem.setFeederPowerCommand(0.0))); 


    // new Trigger(() -> m_visionSubsystem.seesNote())
    //         .whileTrue(new InstantCommand (() -> {
    //     m_operatorController.getHID().setRumble(RumbleType.kRightRumble, 0.2); 
    //   })).onFalse(new InstantCommand (() -> {
    //     m_operatorController.getHID().setRumble(RumbleType.kRightRumble, 0.0); 
    //   }));

    

    // //up arrow -- manual move arm up 
    // m_operatorController.povUp().whileTrue(m_armSubsystem.manualModeCommand(0.08)); 

    // //down arrow -- manual move arm down 
    // m_operatorController.povDown().whileTrue(m_armSubsystem.manualModeCommand(-0.08)); 

    //left trigger -- spin up shooter 
    m_operatorController.leftTrigger().whileTrue(m_shooterSubsystem.setPowerCommand(1.0))
      .onFalse(m_shooterSubsystem.setPowerCommand(0.0)); 

    
    // new Trigger(() -> m_feederSubsystem.getShooterBreaker())
    //   .whileTrue(new InstantCommand (() -> {
    //     m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.2); 
    //   })).onFalse(new InstantCommand (() -> {
    //     m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0); 
    //   }));

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
