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
    public static PathPlannerTrajectory trajectory;
    // Should current pose be used the same way as targetPose
    // (Have a setCurrentPose function?)
    public static Pose2d targetPose, currentPose;
    public static ChassisSpeeds setpoint;
    public static Translation2d translationError;
    public static Rotation2d rotationError;
    public static Supplier<Pose2d> currentPoseSupplier;
    public PathLogger(){}
    
    public void setSources(Supplier<Pose2d> currentPoseSupplier){
        this.currentPoseSupplier = currentPoseSupplier;
        // BeaverLogger.getInstance().addSource("Target X", this::getTargetX);
        // BeaverLogger.getInstance().addSource("Target Y", this::getTargetY);
        // BeaverLogger.getInstance().addSource("Current X", this::getCurrentX);
        // BeaverLogger.getInstance().addSource("Current Y", this::getCurrentY);
    }
    public void logPathProgress(){
        BeaverLogger.getInstance().saveLogs();
    }
    public double getTargetX(){
        if(targetPose != null){
            return targetPose.getX();
        }
        return 0;
    }
    public double getTargetY(){
        if(targetPose != null){
            return targetPose.getY();
        }
        return 0;
    }
    public double getTargetAngle(){
        if(targetPose != null){
            return targetPose.getRotation().getRadians();
        }
        return 0;
    }
    public double getCurrentX(){
        Pose2d pose = currentPoseSupplier.get();
        if(pose != null){
            return pose.getX();
        }
        return 0;        
    }
    public double getCurrentY(){
        Pose2d pose = currentPoseSupplier.get();
        if(pose != null){
            return pose.getY();
        }
        return 0;        
    }
    public double getCurrentAngle(){
        Pose2d pose = currentPoseSupplier.get();
        if(pose != null){
            return pose.getRotation().getRadians();
        }
        return 0;
    }
    public double getXError(){
        if(translationError != null){
            return translationError.getX();
        }
        return 0;
    }
    public double getYError(){
        if(translationError != null){
            return translationError.getY();
        }
        return 0;
    }
    public double getRotationError(){
        if(rotationError != null){
            return rotationError.getRadians();
        }
        return 0;
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