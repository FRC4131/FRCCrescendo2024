package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    public static final class ConveyorConstants {
  
          public static final int conveyorMotor = 8;
          public static final int initialConveyorSensor = 0;
          public static final int finalConveyorSensor = 1;
          public static final int launcherConveyorSensor = 2;
          public static final double conveyorMotorAdvanceSpeed = 0.7;
          public static final double conveyorMotorRetreatSpeed = -0.7;
          public static final double conveyorMotorStopSpeed = 0.0;
  
      }
    private CANSparkMax conveyorMotor;
    private DigitalInput initialConveyorSensor;
    private DigitalInput finalConveyorSensor;
    // Add declaration for launcherConveyorSensor if needed
    
    /**
     * Creates a new Conveyor.
     */
    public Conveyor() {
  
      conveyorMotor = new CANSparkMax(ConveyorConstants.conveyorMotor, MotorType.kBrushless);
      initialConveyorSensor = new DigitalInput(ConveyorConstants.initialConveyorSensor);
      finalConveyorSensor = new DigitalInput(ConveyorConstants.finalConveyorSensor);
      // Initialize launcherConveyorSensor if needed
      
      conveyorMotor.setInverted(true);
  
      conveyorMotor.burnFlash();
    }
  
    public void advance() {
      conveyorMotor.set(ConveyorConstants.conveyorMotorAdvanceSpeed);
    }
  
    public void retreat() {
      conveyorMotor.set(ConveyorConstants.conveyorMotorRetreatSpeed);
    }
  
    public void stop() {
      conveyorMotor.set(ConveyorConstants.conveyorMotorStopSpeed);
    }
  
    public boolean getInitialConveyorSensor() {
      return !initialConveyorSensor.get();
    }
  
    public boolean getFinalConveyorSensor() {
      return !finalConveyorSensor.get();
    }
  
    // Add or remove this method depending on the presence of launcherConveyorSensor
    public boolean getLauncherConveyorSensor() {
      // return !launcherConveyorSensor.get();
      return false; // Temporary placeholder if not implemented
    }
  
    @Override
    public void periodic() {
      SmartDashboard.putBoolean("Intake Sensor", !initialConveyorSensor.get());
      SmartDashboard.putBoolean("Conveyor Sensor", !finalConveyorSensor.get());
      // SmartDashboard.putBoolean("Launcher Sensor", !launcherConveyorSensor.get()); // Uncomment if launcherConveyorSensor is implemented
      // This method will be called once per scheduler run
    }
  }