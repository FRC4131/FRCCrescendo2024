// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_Motor;
  RelativeEncoder m_MotorEncoder;
  SparkPIDController m_MotorPID;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_Motor = new CANSparkMax(2, MotorType.kBrushless);
    m_MotorEncoder = m_Motor.getEncoder();
    // m_MotorPID = m_Motor.getPIDController();


  }

  public void setPower(double power) {
    m_Motor.set(power);
  }

 
  public void resetMotorPos() {
    // m_MotorPID.setP(-1);
  }

  
  
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter",m_Motor.getOutputCurrent());
  }
}
