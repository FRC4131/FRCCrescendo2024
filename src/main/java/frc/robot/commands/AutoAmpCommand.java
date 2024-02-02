// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.PoseEstimationSubsystem;

// public class AutoAmpCommand extends CommandBase {
//     private final DrivetrainSubsystem m_drivetrainSubsystem;
//     private final PoseEstimationSubsystem m_poseEstimationSubsystem;
//     private final PIDController m_pidController;
//     private final DoubleSupplier m_throttle;
//     private Pose2d m_robotPose;
//     private double m_desiredAngle;

//     /** Creates a new AutoAmpCommand. */
//     public AutoAmpCommand(DrivetrainSubsystem drivetrainSubsystem, 
//                           PoseEstimationSubsystem poseEstimationSubsystem, 
//                           DoubleSupplier throttle,
//                           double angle) {
//         m_drivetrainSubsystem = drivetrainSubsystem;
//         m_poseEstimationSubsystem = poseEstimationSubsystem;
//         m_throttle = throttle;
//         m_desiredAngle = angle;

//         m_pidController = new PIDController(4, 0, 0);
//         m_pidController.enableContinuousInput(-Math.PI, Math.PI);
//         addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);

  public AutoAmpCommand(
    DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem, 
    double angle,
    DoubleSupplier x,
    DoubleSupplier y, 
    DoubleSupplier throttle,
    Boolean fieldRelative, 
    Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_desiredAngle = angle;
    m_targetPose = targetPose;

//     @Override
//     public void initialize() {
//         m_pidController.setSetpoint(m_desiredAngle);
//     }

    m_controllerX = x; 
    m_controllerY = y;
    m_throttle = throttle; 
    m_fieldRelative = fieldRelative; 
    m_robotPose = m_poseEstimationSubsystem.getPose();
    }

//     //     // No strafing, only forward and backward movement
//     //     double forwardSpeed = m_throttle.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND;

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_robotPose = m_poseEstimationSubsystem.getPose();
//     m_desiredAngle = Math.atan2(m_targetPose.getY() - m_robotPose.getY(), m_targetPose.getX() - m_robotPose.getX()); 
//     m_pidController.setSetpoint(m_desiredAngle);
//     Double desiredRotation = m_pidController.calculate(m_poseEstimationSubsystem.getPose().getRotation().getRadians());

//         // Move the robot forward/backward with the desired rotation
//         m_drivetrainSubsystem.drive(new Translation2d(0, forwardSpeed),
//                                     desiredRotation,
//                                     m_robotPose.getRotation(),
//                                     true,
//                                     true);
//     }

//     // public void execute() {
//     //   m_robotPose = m_poseEstimationSubsystem.getPose();
//     //   m_desiredAngle = Math.atan2(m_targetPose.getY() - m_robotPose.getY(), m_targetPose.getX() - m_robotPose.getX());
//     //   m_pidController.setSetpoint(m_desiredAngle);
//     //   Double desiredRotation = m_pidController.calculate(m_poseEstimationSubsystem.getPose().getRotation().getRadians());
  
//     //   double throttleValue = m_throttle.getAsDouble();
//     //   double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle
//     //   double forwardSpeed = (slope * throttleValue + Constants.Swerve.MIN_THROTTLE_LEVEL) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
  
//     //   // Drive forward or backward while maintaining the desired rotation
//     //   // Removed the m_controllerX.getAsDouble() for lateral movement to prevent strafing
//     //   m_drivetrainSubsystem.drive(new Translation2d(0, forwardSpeed),
//     //       desiredRotation,
//     //       m_poseEstimationSubsystem.getPose().getRotation(),
//     //       m_fieldRelative,
//     //       true);
//     // }
  

//     @Override
//     public void end(boolean interrupted) {
//         m_drivetrainSubsystem.drive(new Translation2d(), 0, m_robotPose.getRotation(), true, true);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
