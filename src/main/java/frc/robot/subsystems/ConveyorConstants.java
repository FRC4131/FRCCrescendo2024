package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Add this import

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ConveyorConstants {

  public static final int conveyorMotor = 8;
  public static final int initialConveyorSensor = 0;
  public static final int finalConveyorSensor = 1;
  public static final int launcherConveyorSensor = 2;
  public static final double conveyorMotorAdvanceSpeed = 0.7;
  public static final double conveyorMotorRetreatSpeed = -0.7;
  public static final double conveyorMotorStopSpeed = 0.0;
}
