// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import com.fasterxml.jackson.databind.ObjectMapper;

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
      for (int i = 0; i < results.Fiducial.size(); i++) {
        
        if (results.Fiducial.get(i).get("fID") != null && ((int) results.Fiducial.get(i).get("fID")) < 9) {
          int fID = (int)results.Fiducial.get(i).get("fID");
          tags.get(fID).put("tx", results.Fiducial.get(i).get("tx"));
          tags.get(fID).put("ty", results.Fiducial.get(i).get("ty"));
          tags.get(fID).put("Field pose", results.Fiducial.get(i).get("t6r_fs"));
          tags.get(fID).put("Blue bot pose", results.getBotpose_wpiblue());
          tags.get(fID).put("Red bot pose", results.getBotpose_wpired());
          tags.get(fID).put("Tag pose", results.Fiducial.get(i).get("t6t_cs"));
        }
      }
    }
  }
}
