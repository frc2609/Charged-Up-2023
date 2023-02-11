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

/** Add your docs here. */
public class PathLogger {
    public PathPlannerTrajectory trajectory;
    public Pose2d targetPose, currentPose;
    public ChassisSpeeds setpoint;
    public Translation2d translationError;
    public Rotation2d rotationError;
    public PathLogger(Supplier<Pose2d> currentPoseSupplier){
        this.currentPose = currentPoseSupplier.get();
        BeaverLogger.getInstance().addSource("Target X", this.targetPose::getX);
        BeaverLogger.getInstance().addSource("Target Y", this.targetPose::getY);
        BeaverLogger.getInstance().addSource("Current X", this.currentPose::getX);
        BeaverLogger.getInstance().addSource("Current Y", this.currentPose::getY);
        
    }
    public void logPathProgress(){
        BeaverLogger.getInstance().saveLogs();
    }

    public void setActiveTrajectory(PathPlannerTrajectory activeTrajectory){
        this.trajectory = activeTrajectory;
    }
    public void setTargetPose(Pose2d targetPose){
        this.targetPose = targetPose;
    }
    public void setSetpoint(ChassisSpeeds setpoint){
        this.setpoint = setpoint;
    }
    public void setError(Translation2d translationError, Rotation2d rotationError){
        this.translationError = translationError;
        this.rotationError = rotationError;
    }

}
