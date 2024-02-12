package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    public int moduleNumber;
    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private RelativeEncoder m_driveEncoder;
    private CANcoder m_angleEncoder;
    private ProfiledPIDController m_turningPidController;

    public SwerveModule(
        int moduleNumber,
        int angleMotorID, 
        int driveMotorID, 
        int canCoderID,
        double kP) 
        {
        this.moduleNumber = moduleNumber;

        /* Motor & Encoder Config */
        m_angleMotor = new CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        m_angleEncoder = new CANcoder(canCoderID);
        configAngleMotor();
        configDriveMotor();

        // Create module angle PID controller (in SW)
        m_turningPidController = new ProfiledPIDController(kP
        , 0, 0,
                new TrapezoidProfile.Constraints(20 * 2 * Math.PI, 20 * 2 * Math.PI));
        
        // Tell the angle PID controller that it is a *wheel* (i.e. that it wraps from -pi to pi)
        m_turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        m_driveEncoder.setPosition(0);   
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurningPosition()));
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        double percentOutput = (Math.abs(desiredState.speedMetersPerSecond) <= Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01) ? 0
                : desiredState.speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
        m_driveMotor.set(percentOutput);
    }

    private void setAngle(SwerveModuleState desiredState) {
        double desiredPower = m_turningPidController.calculate(getTurningPosition(),
                desiredState.angle.getRadians()); 
        m_angleMotor.set(desiredPower);
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public void rawSet(double drive, double turn) {
        m_driveMotor.set(drive);
        m_angleMotor.set(turn);
    }

    public double getTurningPosition() {
        return m_angleEncoder.getAbsolutePosition().getValueAsDouble() * Constants.Swerve.ANGLE_ENCODER_ROT2RAD; 
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return m_angleEncoder.getVelocity().getValueAsDouble();
    }

    private void configAngleMotor() {
        m_angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
    }

    private void configDriveMotor() {
        m_driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        m_driveMotor.setIdleMode(IdleMode.kBrake); 
        m_driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_ENCODER_ROT2METERS);
        m_driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_ENCODER_RPM2METERSPERSEC);
        m_driveMotor.burnFlash();  //TODO: Do we need to flash these values into the encoder?
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public void reset() {
        m_driveEncoder.setPosition(0);
        m_turningPidController.reset(0);
    }

    public void stop() {
        m_angleMotor.stopMotor();
        m_driveMotor.stopMotor();
    }
}