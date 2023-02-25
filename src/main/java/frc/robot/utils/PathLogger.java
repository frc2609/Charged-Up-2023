// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Collects autonomous path-related data. */
public class PathLogger {
    public static PathPlannerTrajectory m_trajectory;
    public static Pose2d m_targetPose;
    public static Pose2d m_currentPose;
    public static ChassisSpeeds m_setpoint;
    public static Translation2d m_translationError;
    public static Rotation2d m_rotationError;
    public static Supplier<Pose2d> m_currentPoseSupplier;

    public PathLogger() {}
    
    /**
     * Register a Pose2D supplier providing the current pose of the robot.
     * 
     * @param currentPoseSupplier A Supplier<Pose2d> which returns the current
     * position of the robot.
     */
    public void setSources(Supplier<Pose2d> currentPoseSupplier) {
        m_currentPoseSupplier = currentPoseSupplier;
    }
    /**
     * Call currentPoseSupplier if it is registered, or return a new Pose2d if
     * it is not.
     * 
     * @return The current pose of the robot.
     */
    private Pose2d getCurrentPose() {
        if (m_currentPoseSupplier != null) {
            return m_currentPoseSupplier.get();
        } else {
            return new Pose2d();
        }
    }
    
    // unused
    // public void logPathProgress() {
    //     BeaverLogger.getInstance().saveLogs();
    // }

    // Access functions
    public double getTargetX() {
        if (m_targetPose != null) {
            return m_targetPose.getX();
        }
        return 0;
    }
    public double getTargetY() {
        if (m_targetPose != null) {
            return m_targetPose.getY();
        }
        return 0;
    }
    public double getTargetAngle() {
        if (m_targetPose != null) {
            return m_targetPose.getRotation().getRadians();
        }
        return 0;
    }
    public double getCurrentX() {
        Pose2d pose = m_currentPoseSupplier.get();
        if (pose != null) {
            return pose.getX();
        }
        return 0;        
    }
    public double getCurrentY() {
        Pose2d pose = getCurrentPose();
        if (pose != null) {
            return pose.getY();
        }
        return 0;        
    }
    public double getCurrentAngle() {
        Pose2d pose = m_currentPoseSupplier.get();
        if (pose != null) {
            return pose.getRotation().getRadians();
        }
        return 0;
    }
    public double getXError() {
        if (m_translationError != null) {
            return m_translationError.getX();
        }
        return 0;
    }
    public double getYError() {
        if (m_translationError != null) {
            return m_translationError.getY();
        }
        return 0;
    }
    public double getRotationError() {
        if (m_rotationError != null) {
            return m_rotationError.getRadians();
        }
        return 0;
    }

    // Callbacks for PPSwerveControllerCommand.setLoggingCallbacks()
    public void setActiveTrajectory(PathPlannerTrajectory activeTrajectory) {
        m_trajectory = activeTrajectory;
    }
    public void setTargetPose(Pose2d targetPose) {
        m_targetPose = targetPose;
    }
    public void setSetpoint(ChassisSpeeds setpoint) {
        m_setpoint = setpoint;
    }
    public void setError(Translation2d translationError, Rotation2d rotationError) {
        m_translationError = translationError;
        m_rotationError = rotationError;
    }
}