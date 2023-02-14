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
    double getCurrentX(){
        Pose2d pose = currentPoseSupplier.get();
        if(pose != null){
            return pose.getX();
        }
        return 0;        
    }
    double getCurrentY(){
        Pose2d pose = currentPoseSupplier.get();
        if(pose != null){
            return pose.getY();
        }
        return 0;        
    }

    

    public void setActiveTrajectory(PathPlannerTrajectory activeTrajectory){
        this.trajectory = activeTrajectory;
        System.out.println("Recieved trajectory");
    }
    public void setTargetPose(Pose2d targetPose){
        this.targetPose = targetPose;
        System.out.println("Recieved pose");
    }
    public void setSetpoint(ChassisSpeeds setpoint){
        this.setpoint = setpoint;
        System.out.println("Recieved setpoint");
    }
    public void setError(Translation2d translationError, Rotation2d rotationError){
        System.out.println("Recieved error");
        this.translationError = translationError;
        this.rotationError = rotationError;
    }
    

}