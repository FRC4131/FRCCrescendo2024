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
  CANSparkMax m_climberController = new CANSparkMax(0, MotorType.kBrushless);//only uses one motor
  //needs something to release the springs for the extension of the climber
  boolean firstRun = true;
  boolean yoinked = false;

  RelativeEncoder m_climberEncoder;
  SparkMaxPIDController m_climberPID;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberEncoder = m_climberController.getEncoder();
    m_climberPID = m_climberController.getPIDController();

  }

  public void extend(){
    //something here releases the springs
  }

  public void yoink(){
    m_climberController.set(-1); //speed will go here, -1 is a placecholder
    //needs limits so that it stops eventually
    yoinked = true;
  }

  @Override
  public void periodic() {
    if(firstRun){
      yoink();//this will roll up the motors
    }
    // This method will be called once per scheduler run
  }
}
