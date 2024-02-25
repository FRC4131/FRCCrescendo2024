// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax m_shooterMotor; 
  private RelativeEncoder m_shooterEncoder; 
  public double m_speed = 0; 
  public double m_targetRPM = -1100; 

  public ShooterSubsystem() {
    m_shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotor.setSmartCurrentLimit(30); 
    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_shooterMotor.setInverted(true);

    m_shooterEncoder = m_shooterMotor.getEncoder(); 

  }

  public void setPower(double power) {
    m_shooterMotor.set(power);
  }

  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", m_shooterEncoder.getVelocity());
  }
}
