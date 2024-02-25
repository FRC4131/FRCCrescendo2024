package frc.lib.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;

public class EstimatedRobotPose {
    private Pose2d m_pose; 
    private double m_timeStamp; 
    private Vector m_targetPose; 

    public EstimatedRobotPose(Pose2d pose, double timeStamp, Vector vector)
    {
        m_timeStamp = timeStamp; 
        m_pose = pose; 
        m_targetPose = vector; 
    }

    public void setPose(Pose2d pose)
    {
        m_pose = pose; 
    }

    public Pose2d getPose()
    {
        return m_pose;
    }

    public double getTimeStamp()
    {
        return m_timeStamp; 
    }

    public double getMagnitude()
    {
      return m_targetPose.norm(); 
    }
    
}
