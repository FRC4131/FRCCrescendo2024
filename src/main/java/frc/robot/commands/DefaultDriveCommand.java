// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class DefaultDriveCommand extends Command {
  DrivetrainSubsystem m_drivetrainSubsystem;
  PoseEstimationSubsystem m_poseEstimationSubsystem;
  DoubleSupplier x;
  DoubleSupplier y;
  DoubleSupplier theta;
  DoubleSupplier throttle;
  boolean fieldRelative;

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimationSubsystem poseEstimationSubsystem,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier theta,
      DoubleSupplier throttle,
      boolean fieldRelative) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.fieldRelative = fieldRelative;
    this.throttle = throttle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    addRequirements(poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //DataLogManager.log("DRIVE init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL;
    double scale = slope * this.throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL;
    m_drivetrainSubsystem.drive(new Translation2d(x.getAsDouble() * scale,
        y.getAsDouble() * scale),
        theta.getAsDouble() * scale,
        m_poseEstimationSubsystem.getPose().getRotation(),
        fieldRelative,
        true);

      //  m_drivetrainSubsystem.drive(new Translation2d(x.getAsDouble() * scale,
      //   0.0),
      //   theta.getAsDouble() * scale,
      //   m_poseEstimationSubsystem.getPose().getRotation(),
      //   fieldRelative,
      //   true);

    
    //DataLogManager.log("DRIVE EXECUTE");
    SmartDashboard.putNumber("x axis", x.getAsDouble()); 
    SmartDashboard.putNumber("y axis", y.getAsDouble()); 
    SmartDashboard.putNumber("rot axis", theta.getAsDouble()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), fieldRelative, true);
    //DataLogManager.log("DRIVE END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
