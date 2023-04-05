package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.kinematics.SwerveModuleState;
//taken from https://github.com/TripleHelixProgramming/HelixUtilities
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.SwerveModule;

/** Records values into a .csv file for later viewing. */
public class BeaverLogger {
	private static BeaverLogger INSTANCE = new BeaverLogger();

	public static BeaverLogger getInstance() {
		return INSTANCE;
	}

	private final List<LogSource> dataSources = new ArrayList<>();
	private Path file;
	
	private String loggingLocation = "/home/lvuser/logs/";
	
	private BeaverLogger() {
		File usb1 = new File("/media/sda1/");
		if (usb1.exists()) {
			loggingLocation = "/media/sda1/logs/";
		}
	}
	
	
	private void createLogDirectory() throws IOException {
		File logDirectory = new File(loggingLocation);
		if (!logDirectory.exists()) {
			Files.createDirectory(Paths.get(loggingLocation));
		}
	}
	
	private void createFile() {
		Writer output = null;
		try {
			createLogDirectory();
      		String fileName = "beaverlog";
      		int logNumber = 0;
			file = Paths.get(loggingLocation + fileName+Integer.toString(logNumber)+".csv");
			while (Files.exists(file)) {
        		logNumber++;
        		file = Paths.get(loggingLocation + fileName+Integer.toString(logNumber)+".csv");
			}
			Files.createFile(file);
			saveTitles();
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			if (output != null) {
				try {
					output.close();
				} catch (IOException e) {}
			}
		}
	}

	// unused
	public void addSource(String name, Supplier<Object> supplier) {
		dataSources.add(new LogSource(name, supplier));
	}
	
	// unused (used in unused function)
	public void saveLogs() {
		try {
			if (file == null) {
				createFile();
			}

			StringBuilder data = new StringBuilder();
			data.append(Instant.now().toString()).append(",");
			data.append(Timer.getFPGATimestamp()).append(",");
			data.append(getValues());
			Files.write(file, Collections.singletonList(data.toString()), StandardOpenOption.APPEND);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void logMP(PathLogger path, SwerveModuleState[] targetStates, SwerveModuleState[] currentStates, SwerveModule module) {
		try {
			if (file == null) {
				createFile();
			}

			StringBuilder data = new StringBuilder();
			data.append(Instant.now().toString()).append(",");
			data.append(Timer.getFPGATimestamp()).append(",");
			data.append(Double.toString(path.getCurrentX()) + ',');
			data.append(Double.toString(path.getCurrentY()) + ',');
			data.append(Double.toString(path.getCurrentAngle()) + ',');
			data.append(Double.toString(path.getTargetX()) + ',');
			data.append(Double.toString(path.getTargetY()) + ',');
			data.append(Double.toString(path.getTargetAngle()) + ',');
			
			data.append(Double.toString(currentStates[0].speedMetersPerSecond) + ',');
			data.append(Double.toString(targetStates[0].speedMetersPerSecond) + ',');
			data.append(Double.toString(currentStates[1].speedMetersPerSecond) + ',');
			data.append(Double.toString(targetStates[1].speedMetersPerSecond) + ',');
			data.append(Double.toString(currentStates[2].speedMetersPerSecond) + ',');
			data.append(Double.toString(targetStates[2].speedMetersPerSecond) + ',');
			data.append(Double.toString(currentStates[3].speedMetersPerSecond) + ',');
			data.append(Double.toString(targetStates[3].speedMetersPerSecond) + ',');

			data.append(Double.toString(currentStates[0].angle.getDegrees()) + ',');
			data.append(Double.toString(targetStates[0].angle.getDegrees()) + ',');
			data.append(Double.toString(currentStates[1].angle.getDegrees()) + ',');
			data.append(Double.toString(targetStates[1].angle.getDegrees()) + ',');
			data.append(Double.toString(currentStates[2].angle.getDegrees()) + ',');
			data.append(Double.toString(targetStates[2].angle.getDegrees()) + ',');
			data.append(Double.toString(currentStates[3].angle.getDegrees()) + ',');
			data.append(Double.toString(targetStates[3].angle.getDegrees()) + ',');
			data.append(Double.toString(module.getSecondaryVelocity()));
			
			Files.write(file, Collections.singletonList(data.toString()), StandardOpenOption.APPEND);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void logArm(double[] setpoint, ArmGripper arm) {
		try {
			if (file == null) {
				createFile();
			}

			StringBuilder data = new StringBuilder();
			data.append(Instant.now().toString()).append(",");
			data.append(Timer.getFPGATimestamp()).append(",");
			data.append(Double.toString(setpoint[0]) + ',');
			data.append(Double.toString(setpoint[1]) + ',');
			data.append(Double.toString(setpoint[2]) + ',');

			data.append(Double.toString(arm.getLowerAngleRelative()) + ',');
			data.append(Double.toString(arm.getLowerJointAngularVelocity()) + ',');
			
			data.append(Double.toString(arm.getUpperAngleRelative()) + ',');
			data.append(Double.toString(arm.getUpperJointAngularVelocity()) + ',');

			data.append(Double.toString(arm.getExtensionDistance()) + ',');
			data.append(Double.toString(arm.getExtensionVelocity()) + ',');

			// data.append(Double.toString(currentStates[0].angle.getDegrees()) + ',');
			// data.append(Double.toString(targetStates[0].angle.getDegrees()) + ',');
			// data.append(Double.toString(currentStates[1].angle.getDegrees()) + ',');
			// data.append(Double.toString(targetStates[1].angle.getDegrees()) + ',');
			// data.append(Double.toString(currentStates[2].angle.getDegrees()) + ',');
			// data.append(Double.toString(targetStates[2].angle.getDegrees()) + ',');
			// data.append(Double.toString(currentStates[3].angle.getDegrees()) + ',');
			// data.append(Double.toString(targetStates[3].angle.getDegrees()) + ',');
			Files.write(file, Collections.singletonList(data.toString()), StandardOpenOption.APPEND);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	private void saveTitlesArm() throws IOException {
		StringBuilder titles = new StringBuilder();
		titles.append("Timestamp,");
		titles.append("match_time,");
		titles.append("Lower setp,");
		titles.append("Upper setp,");
		titles.append("Ext setp,");
		titles.append("Lower pos,");
		titles.append("Lower vel,");
		titles.append("Upper pos,");
		titles.append("Upper vel,");
		titles.append("Extension pos,");
		titles.append("Extension vel,");
		titles.append(dataSources.stream().map(t -> t.name).collect(Collectors.joining(","))).append(",");
		Files.write(file, Collections.singletonList(titles.toString()), StandardOpenOption.APPEND);
	}

	private void saveTitles() throws IOException {
		StringBuilder titles = new StringBuilder();
		titles.append("Timestamp,");
		titles.append("match_time,");
		titles.append("Current X,");
		titles.append("Current Y,");
		titles.append("Current Angle,");
		titles.append("Target X,");
		titles.append("Target Y,");
		titles.append("Target Angle,");
		titles.append("Curr FL,");
		titles.append("Target FL,");
		titles.append("Curr FR,");
		titles.append("Target FR,");
		titles.append("Curr RL,");
		titles.append("Target RL,");
		titles.append("Curr RR,");
		titles.append("Target RR,");
		titles.append("Curr FLR,");
		titles.append("Target FLR,");
		titles.append("Curr FRR,");
		titles.append("Target FRR,");
		titles.append("Curr RLR,");
		titles.append("Target RLR,");
		titles.append("Curr RRR,");
		titles.append("Target RRR,");

		titles.append(dataSources.stream().map(t -> t.name).collect(Collectors.joining(","))).append(",");
		Files.write(file, Collections.singletonList(titles.toString()), StandardOpenOption.APPEND);
	}

	// unused (used in unused function)
	private String getValues() {
		return dataSources.stream().map(s -> s.supplier.get()).map(Object::toString).collect(Collectors.joining(","));
	}

	private class LogSource {
		private final String name;
		private final Supplier<Object> supplier;

		public LogSource(String name, Supplier<Object> supplier) {
			this.name = name;
			this.supplier = supplier;
		}
	}
}
