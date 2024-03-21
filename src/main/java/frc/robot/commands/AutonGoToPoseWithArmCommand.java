// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class AutonGoToPoseWithArmCommand extends Command {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private PoseEstimationSubsystem m_poseEstimationSubsystem;
  private ArmSubsystem m_armSubsystem; 
  private Double m_desiredDriveAngle;
  private Double m_desiredArmAngleDegrees; 
  private DoubleSupplier m_controllerX; 
  private DoubleSupplier m_controllerY; 
  private DoubleSupplier m_throttle; 
  private Boolean m_fieldRelative;
  private Pose2d m_robotPose; 
  private Supplier<Pose2d> m_targetPose; 

    Double xDistance;
    Double yDistance;
    Double totalDistance;
  PIDController m_pidControllerDrive;
  //PIDController m_pidControllerArm; 

  /** Creates a new FixateOnThing. */
  public AutonGoToPoseWithArmCommand(DrivetrainSubsystem drivetrainSubsystem, 
    ArmSubsystem armSubsystem,
    PoseEstimationSubsystem poseEstimationSubsystem, 
    double angle,
    DoubleSupplier x,
    DoubleSupplier y, 
    DoubleSupplier throttle,
    Boolean fieldRelative, 
    Supplier<Pose2d> targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_armSubsystem = armSubsystem;
    m_desiredDriveAngle = angle;
    m_targetPose = targetPose;

    m_pidControllerDrive = new PIDController(4, 0, 0);
    m_pidControllerDrive.enableContinuousInput(-Math.PI, Math.PI);
      m_pidControllerDrive.setTolerance(2);
    addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem, m_armSubsystem);

  
    m_controllerX = x; 
    m_controllerY = y;
    m_throttle = throttle; 
    m_fieldRelative = fieldRelative; 
    m_robotPose = m_poseEstimationSubsystem.getPose();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //DataLogManager.log("Auton Arm START");
    Pose2d targetPose = m_targetPose.get();
    xDistance = targetPose.getX() - m_robotPose.getX();
    yDistance = targetPose.getY() - m_robotPose.getY(); 
    totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); 

    m_robotPose = m_poseEstimationSubsystem.getPose();
    m_desiredDriveAngle = Math.atan2(yDistance, xDistance); //angle computed for the bot driving 
    m_desiredArmAngleDegrees = Math.atan2(Constants.FieldConstants.SPEAKER_HEIGHT_METERS + m_armSubsystem.getOffset(), totalDistance) * (180/Math.PI); //angle computed for the arm 
    m_pidControllerDrive.setSetpoint(m_desiredDriveAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //DataLogManager.log("Auton Arm EXECUTE");
    //l2 norm for distance between bot and speaker
    Pose2d targetPose = m_targetPose.get(); 
    xDistance = targetPose.getX() - m_robotPose.getX();
    yDistance = targetPose.getY() - m_robotPose.getY(); 
    totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); 

    m_robotPose = m_poseEstimationSubsystem.getPose();
    m_desiredDriveAngle = Math.atan2(yDistance, xDistance); //angle computed for the bot driving 
    m_desiredArmAngleDegrees = Math.atan2(Constants.FieldConstants.SPEAKER_HEIGHT_METERS, totalDistance) * (180/Math.PI); //angle computed for the arm 
    m_pidControllerDrive.setSetpoint(m_desiredDriveAngle);
    //m_pidControllerArm.setSetpoint(m_desiredArmAngle);
    Double desiredRotationDrive = m_pidControllerDrive.calculate(m_poseEstimationSubsystem.getPose().getRotation().getRadians());
    //Double desiredRotationArm = m_pidControllerArm.calculate(m_armSubsystem.getArmAngle()); 

    double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle 
    double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL; 
    m_drivetrainSubsystem.drive(new Translation2d(m_controllerX.getAsDouble() * scale,
        m_controllerY.getAsDouble() * scale),
        desiredRotationDrive,
        m_poseEstimationSubsystem.getPose().getRotation(),
        m_fieldRelative,
        true);
    if (m_desiredArmAngleDegrees >= Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE && m_desiredArmAngleDegrees < 90)
    {
       m_armSubsystem.goToAngle(m_desiredArmAngleDegrees); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
    //DataLogManager.log("Auton Arm END");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return false;
  }
}
