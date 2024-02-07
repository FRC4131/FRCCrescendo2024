// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax m_climberController = new CANSparkMax(0, MotorType.kBrushless);//get ID later
  private RelativeEncoder m_climberEncoder;
  private SparkMaxPIDController m_climberPID;
  boolean firstRun = true;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberEncoder = m_climberController.getEncoder();
    m_climberPID = m_climberController.getPIDController();
  }

  public void setClimberSpeed(double speed){
    m_climberPID.setReference(speed, ControlType.kDutyCycle); //unsure about control type
  }
  public void extend(){
    //the climber gets released
  }

  public void yoink(){
    while(//motor is not all the way pulled in);
    {
      //pull in motor
    }
    //have the motor pull in the climber, use limits
  }

  @Override
  public void periodic() {
    if(firstRun){
      yoink();
    }
    // This method will be called once per scheduler run
  }
}
