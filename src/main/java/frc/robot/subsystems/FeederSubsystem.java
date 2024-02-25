// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonFormat.Feature;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederState;

public class FeederSubsystem extends SubsystemBase {
  private CANSparkMax m_IntakeMotor;
  private CANSparkMax m_BeltMotor;
  private DigitalInput m_Breaker;
  // private DigitalInput m_FeederBreaker;

  
  // machine states are located in the Constants file

    
  //private IntakeSubsystem m_intakeSubsystem;
  //private FeederSubsystem m_feederSubsystem;
    //private ShooterSubsystem m_shooterSubsystem;
    private FeederState m_State;
    private int m_ShooterTicks = 0;

    private static final int EXECUTE_MS = 20;
    private static final int SHOOT_DURATION_MS = 1000;
    private static final int SHOOTER_TICKS = SHOOT_DURATION_MS / EXECUTE_MS;


  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
      m_IntakeMotor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
      m_BeltMotor = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);
      m_Breaker = new DigitalInput(2); // beambreak
      m_State = FeederState.INPUT;
      m_ShooterTicks = 0;
      m_IntakeMotor.set(0.0);
      m_BeltMotor.set(0.0);
  }


  public void setIntakePower(double power) {
    m_IntakeMotor.set(power);
    m_BeltMotor.set(power);
  }
  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setIntakePower(power);
    }, this);

  }

  public Command setFeederPowerCommand(double power) {
    return new InstantCommand(() -> {
      m_BeltMotor.set(power);
    }, this);
  } 

  public boolean getBreaker() {
    return m_Breaker.get();
  }

  /*public boolean getFeederBreaker() { 
    return m_FeederBreaker.get();
  } */

  public FeederState getFeederState() {
    return m_State;
  }

  public boolean intakeAllowed(){
    return (m_State == FeederState.INPUT);
    // the driver will just have to know that only one note can be in the robot at a time
    // we don't have the first breaker so this is automated now!
  }

  public boolean shootAllowed() {
    return (m_State == FeederState.SHOOT);
    // this remains the same though ;)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("FeederState", m_State.name());
    SmartDashboard.putBoolean("BeamBreak", m_Breaker.get());

    switch (m_State) {

      case INPUT:
          // only 1 beam break, so nothing happens here
          if (m_Breaker.get()) {
            // if the breaker at the shoot turns on then we should be in the shoot state
              m_State = FeederState.SHOOT;
          }
          break;
    
      case SHOOT:
          // stop feeding, keep note in the top of the feeder, so it is ready to shoot (it won't slide)
          m_BeltMotor.set(0.0);
          if (!m_Breaker.get()) {
              m_State = FeederState.INPUT;
          }
          // once the breaker is false again (in other words, shoot has happened, then the states reset)
          // shooting is an external action which can be found in the shooter subsystem.
          break;

      /*case SHOOT:
          // feeder continue running

          m_shooterSubsystem.setPower(0.04);
          m_ShooterTicks++;

          // ok im trying to make it last for 1 second but this doesn't work well :(
          if (m_ShooterTicks >= SHOOTER_TICKS) {
              m_State = State.DONE;
              m_shooterSubsystem.setPower(0);
              m_feederSubsystem.setPower(0);
          }*/
          

          
  }
  }
}
