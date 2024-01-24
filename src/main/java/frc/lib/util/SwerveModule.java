package frc.lib.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;

    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private RelativeEncoder driveEncoder;
    private CANcoder angleEncoder;
    private ProfiledPIDController turningPidController;

    public SwerveModule(
        int moduleNumber,
        int angleMotorID, 
        int driveMotorID, 
        int canCoderID ) 
        {
        this.moduleNumber = moduleNumber;

        /* Motor & Encoder Config */
        m_angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = m_driveMotor.getEncoder();
        angleEncoder = new CANcoder(canCoderID);

        configAngleMotor();
        configDriveMotor();

        // Create PID controller on ROBO RIO
        turningPidController = new ProfiledPIDController(0.4, 0, 0,
                new TrapezoidProfile.Constraints(20 * 2 * Math.PI, 20 * 2 * Math.PI));

        // Tell PID controller that it is a *wheel*
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        lastAngle = getState().angle;

        // angleEncoder.configMagnetOffset(moduleConstants.angleOffset.getDegrees());
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(0);
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
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 0.001))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_angleMotor.set(turningPidController.calculate(getTurningPosition(),
                desiredState.angle.getRadians()));
        lastAngle = angle;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public void rawSet(double drive, double turn) {
        m_driveMotor.set(drive);
        m_angleMotor.set(turn);
    }

    public double getTurningPosition() {
        return angleEncoder.getAbsolutePosition().getValue();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return angleEncoder.getVelocity().getValue();
        // return angleEncoder.getVelocity();    
    }

     private void configAngleMotor() {
         m_angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
     }

    private void configDriveMotor() {
        m_driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        // m_driveMotor.setIdleMode(IdleMode.kBrake); 
        driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_ENCODER_ROT2METERS);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_ENCODER_RPM2METERSPERSEC);
        m_driveMotor.burnFlash();  //TODO: Do we need to flash these values into the encoder?
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public void reset() {
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(0);
        turningPidController.reset(0);
    }

    public void stop() {
        m_angleMotor.stopMotor();
        m_driveMotor.stopMotor();
    }
}