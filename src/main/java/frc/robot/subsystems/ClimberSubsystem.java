// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax m_climberControllerRight = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax m_climberControllerLeft = new CANSparkMax(0, MotorType.kBrushless);
  boolean firstRun = true;

  RelativeEncoder m_climberEncoder;
  SparkMaxPIDController m_climberPID;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberEncoder = m_climberControllerRight.getEncoder();
    m_climberPID = m_climberControllerRight.getPIDController();

  }

  public void extend(){
    m_climberControllerLeft.set(1); //speed will go here, 1 is a placecholder
    m_climberControllerRight.follow(m_climberControllerLeft);//will make this one follow the left motor's actions, might need inversions
  }

  public void yoink(){
    m_climberControllerLeft.set(-1); //speed will go here, -1 is a placecholder
    m_climberControllerRight.follow(m_climberControllerLeft);
  }

  @Override
  public void periodic() {
    if(firstRun){
      yoink();
    }
    // This method will be called once per scheduler run
  }
}
