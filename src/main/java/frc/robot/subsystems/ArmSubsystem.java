// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

//import beam break and motor susbsytems (look for later)

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_beltMotor;
  private DigitalInput firstBreaker;
  private DigitalInput secondBreaker; 
  /** Creates a new SwivelArmSubsystem. */
  public ArmSubsystem() {


  }
//shooting subsystem 
  public void ShootSubsystem () {
    firstBreaker = new DigitalInput(0);
    secondBreaker = new DigitalInput(0);
    m_beltMotor.set(0.0);

    if 
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
