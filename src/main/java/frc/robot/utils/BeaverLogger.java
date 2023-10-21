// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.function.Supplier;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class BeaverLogger {
    HashMap<String, Supplier<Double>> loggingMap = new HashMap<String, Supplier<Double>>();
	HashMap<String, Supplier<Double>> smartDashboardMap = new HashMap<String, Supplier<Double>>();
	HashMap<String, DoubleLogEntry> logEntryMap = new HashMap<String, DoubleLogEntry>();
	final DataLog log = DataLogManager.getLog();
	public BeaverLogger(){
    	for (String key : loggingMap.keySet()) {
			logEntryMap.put(key, new DoubleLogEntry(log, key));
		}
	}
	public void addLoggable(String name, Supplier<Double> supplier, boolean toSmartDash){
		if(toSmartDash){
			smartDashboardMap.put(name, supplier);
		}else{
			loggingMap.put(name, supplier);
		}
	}
	public void logAll(){
    	// Logging values in smartDashboardMap
    	for (String key : smartDashboardMap.keySet()) {
    	    Double value = smartDashboardMap.get(key).get();
			SmartDashboard.putNumber(key, value);
    	    // You can use your actual method for sending data to SmartDashboard here
    	}
    	// Logging values in loggingMap
    	for (String key : loggingMap.keySet()) {
    	    Double value = loggingMap.get(key).get();
			DoubleLogEntry entry = logEntryMap.get(key);
			if(entry == null){
				DataLogManager.log(String.format("key: %s not found in logEntryMap", key));
			}else{
				entry.append(value);
			}
    	}
	}
}


/*
 * 
  private class Log {
    private DoubleLogEntry lowerAngle, upperAngle, extensionDistance,
                           lowerBackupAngle, upperBackupAngle,
                           lowerSetpoint, upperSetpoint, extensionSetpoint,
                           lowerOutput, upperOutput, extensionOutput,
                           lowerCurrent, upperCurrent, extensionCurrent;
    
    private Arm arm;

    public Log(Arm arm) {
      final DataLog log = DataLogManager.getLog();
      lowerSetpoint = new DoubleLogEntry(log, "/arm/setpoints/lower");
      upperSetpoint = new DoubleLogEntry(log, "/arm/setpoints/upper");
      extensionSetpoint = new DoubleLogEntry(log, "/arm/setpoints/extension");
  
      lowerAngle = new DoubleLogEntry(log, "/arm/angles/lower");
      upperAngle = new DoubleLogEntry(log, "/arm/angles/upper");
      extensionDistance = new DoubleLogEntry(log, "/arm/angles/extension");
  
      lowerBackupAngle = new DoubleLogEntry(log, "arm/angles/lower_backup");
      upperBackupAngle = new DoubleLogEntry(log, "arm/angles/upper_backup");
  
      lowerOutput = new DoubleLogEntry(log, "/arm/output/lower");
      upperOutput = new DoubleLogEntry(log, "/arm/output/upper");
      extensionOutput = new DoubleLogEntry(log, "/arm/output/extension");
  
      lowerCurrent = new DoubleLogEntry(log, "/arm/current/lower");
      upperCurrent = new DoubleLogEntry(log, "/arm/current/upper");
      extensionCurrent = new DoubleLogEntry(log, "/arm/current/extension");
      
      this.arm = arm;
    }
    
    public void logAll(){
      lowerAngle.append(arm.getLowerAngle().getDegrees());
      upperAngle.append(arm.getUpperAngle().getDegrees());
      extensionDistance.append(arm.getExtensionDistance());
      lowerBackupAngle.append(arm.getLowerBackupAngle().getDegrees());
      upperBackupAngle.append(arm.getUpperBackupAngle().getDegrees());
      lowerOutput.append(1.0); // current PID output
      upperOutput.append(1.0); // current PID output
      extensionOutput.append(1.0); // current PID output

    }
    public void setCurrentSuppliers(){
      
    }
  }

 */