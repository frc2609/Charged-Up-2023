// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimeLightJSON_GS;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Results;

public class Limelight extends SubsystemBase {
  String limelightJSON;
  LimeLightJSON_GS limelight;
  ObjectMapper mapper;
  Results results;
  //Make sure the hashmap at the index is not null when calling
  public ArrayList<HashMap<String, Object>> tags = new ArrayList<>(9);
  
  // Indexes from lowest to highest are as follows: x, y, z, roll, pitch, yaw (coordinates in meters, rotation in degrees)
  private float[] botpose_red = new float[6];
  private float[] botpose_blue = new float[6];
  private boolean tags_visible;

  /** Creates a new Limelight. */
  public Limelight() {
    for (int i = 0; i < 8; i++) {
      HashMap<String, Object> hMaps = new HashMap<>();
      tags.add(i, hMaps);
    }
    mapper = new ObjectMapper();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightJSON = LimelightHelpers.getJSONDump("limelight");
    try {
      limelight = mapper.readValue(limelightJSON, LimeLightJSON_GS.class);
      results = limelight.getResults();
    } catch (Exception e) {
      // TODO: handle exception
      System.out.println(e);
    }
    if (results != null) {
      botpose_blue = results.getBotpose_wpiblue();
      botpose_red = results.getBotpose_wpired();
      tags_visible = results.getV();
      for (int i = 0; i < results.Fiducial.size(); i++) {
        
        if (results.Fiducial.get(i).get("fID") != null && ((int) results.Fiducial.get(i).get("fID")) < 9) {
          int fID = (int)results.Fiducial.get(i).get("fID");
          tags.get(fID).put("tx", results.Fiducial.get(i).get("tx"));
          tags.get(fID).put("ty", results.Fiducial.get(i).get("ty"));
          tags.get(fID).put("Field pose", results.Fiducial.get(i).get("t6r_fs"));
          tags.get(fID).put("Tag pose", results.Fiducial.get(i).get("t6t_cs"));
          
        }
      }
    }
  }

  /**
   * Check the current alliance colour.
   * 
   * @return Whether the alliance is blue or not. Returns false (red) if alliance is invalid.
   */
  private boolean isBlueAlliance() {
    switch (DriverStation.getAlliance()) {
      case Blue:
        return true;
      case Red:
        return false;
      default:
        System.out.print("Failed to get valid alliance colour");
        return false;
    }
  }

  private Pose2d toPose2d(float[] array) {
    return new Pose2d(array[0], array[1], Rotation2d.fromDegrees(array[5]));
  }

  public boolean isPoseValid() {
    return tags_visible;
  }

  public Pose2d getNodePose() {
    if (isBlueAlliance()) {

    } else {

    }
    // alliance
    // blue: origin is left, nodes start on right
    // red: origin is left, nodes start on left
    // what happens when we don't see any apriltags?
    // need at least one of any apriltags on this side
  }

  public Pose2d getRobotPose() {
    return toPose2d(isBlueAlliance() ? botpose_blue : botpose_red);
  }
}
