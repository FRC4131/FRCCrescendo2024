// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class AutonSpeakerAlignmentCommand extends Command {
  private DrivetrainSubsystem m_DrivetrainSubsystem; 
  private PoseEstimationSubsystem m_PoseEstimationSubsystem; 
  private ArmSubsystem m_ArmSubsystem; 
  private PIDController m_drivePID; 
  private Double m_desiredDriveAngle;
  private Double m_desiredArmAngleDegrees; 
  private Pose2d m_robotPose; 
  private Pose2d m_targetPose; 

  /** Creates a new AutonSpeakerAlignmentCommand. */
  public AutonSpeakerAlignmentCommand(DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem, 
    ArmSubsystem armSubsystem, 
    Pose2d targetPose)  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem; 
    m_PoseEstimationSubsystem = poseEstimationSubsystem; 
    m_ArmSubsystem = armSubsystem; 
    m_drivePID = new PIDController(4, 0, 0); 
    m_drivePID.enableContinuousInput(-Math.PI, Math.PI);
    m_robotPose = m_PoseEstimationSubsystem.getPose(); 
    addRequirements(m_DrivetrainSubsystem, m_PoseEstimationSubsystem, m_ArmSubsystem);
    m_targetPose = targetPose; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double xDistance = m_targetPose.getX() - m_robotPose.getX();
    Double yDistance = m_targetPose.getY() - m_robotPose.getY(); 
    Double totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); 

    m_robotPose = m_PoseEstimationSubsystem.getPose();
    m_desiredDriveAngle = Math.atan2(yDistance, xDistance); //angle computed for the bot driving 
    m_desiredArmAngleDegrees = Math.atan2(Constants.FieldConstants.SPEAKER_HEIGHT_METERS, totalDistance) * (180/Math.PI); //angle computed for the arm 
    m_drivePID.setSetpoint(m_desiredDriveAngle);
    //m_pidControllerArm.setSetpoint(m_desiredArmAngle);
    Double desiredRotationDrive = m_drivePID.calculate(m_PoseEstimationSubsystem.getPose().getRotation().getRadians());
    //Double desiredRotationArm = m_pidControllerArm.calculate(m_armSubsystem.getArmAngle()); 

    m_DrivetrainSubsystem.drive(new Translation2d(0,0),
        desiredRotationDrive,
        m_PoseEstimationSubsystem.getPose().getRotation(),
        true,
        true);
        
    if (m_desiredArmAngleDegrees >= Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE && m_desiredArmAngleDegrees < 90)
    {
       m_ArmSubsystem.goToAngle(m_desiredArmAngleDegrees); 
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
