package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class EstimatedRobotPose {
    private Pose2d m_pose; 
    private double m_timeStamp; 
    public EstimatedRobotPose(Pose2d pose, double timeStamp)
    {
        m_timeStamp = timeStamp; 
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
    
}
