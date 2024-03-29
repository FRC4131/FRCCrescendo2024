// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetAmpCommand extends Command {
  /** Creates a new TargetAmpCommand. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private PoseEstimationSubsystem m_poseEstimationSubsystem;
  private VisionSubsystem m_visionSubsystem; 
  private ArmSubsystem m_armSubsystem; 
  
  Double m_desiredAngle;
  private DoubleSupplier m_controllerY; 
  private DoubleSupplier m_controllerX; 
  private DoubleSupplier m_throttle; 
  private Boolean m_fieldRelative;
  private Pose2d m_robotPose; 
  private Supplier<Pose2d> m_targetPose; 

  private PIDController m_pidControllerTheta;
  private PIDController m_xController; 

  public TargetAmpCommand(DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem,  
    ArmSubsystem armSubsystem, 
    VisionSubsystem visionSubsystem, 
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier, 
    DoubleSupplier throttle, 
    Boolean fieldRelative, 
    Supplier<Pose2d> targetPose) {

      m_drivetrainSubsystem = drivetrainSubsystem;
      m_poseEstimationSubsystem = poseEstimationSubsystem;
      m_armSubsystem = armSubsystem; 
      m_visionSubsystem = visionSubsystem; 
      m_targetPose = targetPose;
      m_pidControllerTheta = new PIDController(4, 0, 0);
      m_pidControllerTheta.enableContinuousInput(-Math.PI, Math.PI);

       m_xController = new PIDController(3, 0, 0); 

        //m_yController = new ProfiledPIDController(3, 0, 0,
        //new Constraints(1.0, 1.0));

      addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem, m_armSubsystem);
  
      m_controllerX = xSupplier; 
      m_controllerY = ySupplier;
      m_throttle = throttle; 
      m_fieldRelative = fieldRelative; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose = m_targetPose.get();
    m_xController.reset(); 
    //m_xController.setSetpoint(targetPose.getX());
    m_pidControllerTheta.setSetpoint((Math.PI / 2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Optional<Double> txOffset = m_visionSubsystem.getAmpOffset(); 
    m_robotPose = m_poseEstimationSubsystem.getPose();
    double pidDesiredRotation = m_pidControllerTheta.calculate(m_robotPose.getRotation().getRadians());
    //double pidDesiredX = m_xController.calculate(m_robotPose.getX());
    // double pidDesiredX = 0.0; 
    // if (txOffset.isPresent()) // if the robot sees a note
    // {
    //   m_xController.setSetpoint(0.0); // goal: tx == 0
    //   pidDesiredX = m_xController.calculate(txOffset.get()); // gets tx and converts to radians
    // }
    // Calculate the throttle scaling factor
    double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle 
    double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL; 

        m_armSubsystem.goToAngle(Constants.ArmConstants.ARM_AMP_ANGLE);

        m_drivetrainSubsystem.drive(new Translation2d(m_controllerX.getAsDouble() * scale,
        m_controllerY.getAsDouble() * scale),
        pidDesiredRotation,
        m_poseEstimationSubsystem.getPose().getRotation(),
        m_fieldRelative,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds());
    m_armSubsystem.goToAngle(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
