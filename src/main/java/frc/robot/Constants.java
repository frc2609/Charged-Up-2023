// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import static java.util.Map.entry;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TimerDelay;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Arm and Gripper-related Constants */
    public static final class Arm {
        public static final class IsInverted {
            public static final boolean LOWER_MOTOR = false;
            public static final boolean UPPER_MOTOR = false;
            public static final boolean EXTENSION_MOTOR = false;
        }
        public static final class Encoder {
            /** How many metres the extension extends per motor rotation. */
            public static final double EXTENSION_POSITION_CONVERSION = Ratios.EXTENSION_MOTOR * EXTENSION_PULLEY_CIRCUMFERENCE; // metres
            public static final double LOWER_DISTANCE_PER_ROTATION = (18/48); // TODO: determine units
            public static final double UPPER_DISTANCE_PER_ROTATION = (18/48); // TODO: determine units
            /** Pointing straight up (angle = 90.0 degrees). */
            public static final double LOWER_POSITION_OFFSET = 0.8537;
            // encoder + dir
            /** Parallel with upper arm (angle = 90 degrees). */
            public static final double UPPER_POSITION_OFFSET = 0.925;
            // encoder + dir (down is negative), away from front positive
            // 0.9305 0 degree setpoint
            // negative forward for both encoders == encoder is ccw pos when viewed from right side = it go in right dir
            // away from front positive
        }
        public static final class Pneumatics {
            public static final int OPEN_SOLENOID_ID = 14;
            public static final int CLOSE_SOLENOID_ID = 15;
        }
        public static final class Ratios {
            /** Extension pulley rotations per extension motor rotation. */
            public static final double EXTENSION_MOTOR = UltraPlanetaryRatios.FIVE_TO_ONE * UltraPlanetaryRatios.FOUR_TO_ONE;
            /** Ratio between shaft pulley (18t) and arm pulley (48t). */
            public static final double LOWER_ARM_CHAIN = 18.0 / 48.0;
            /** Arm shaft rotations per motor rotation. (2x 5:1 + 1x 3:1) */
            public static final double LOWER_ARM_MOTOR = (1.0 / 5.0) * (1.0 / 5.0) * (1.0 / 3.0);
            /** Total ratio between lower arm motor and lower arm rotation. */
            public static final double LOWER_ARM = LOWER_ARM_MOTOR * LOWER_ARM_CHAIN;
            /** Ratio between shaft pulley (18t) and arm pulley (48t). */
            public static final double UPPER_ARM_CHAIN = 18.0 / 48.0;
            /** Arm shaft rotations per motor rotation. (2x 5:1 + 1x 3:1) */
            public static final double UPPER_ARM_MOTOR = (1.0 / 5.0) * (1.0 / 5.0) * (1.0 / 3.0);
            /** Total ratio between upper arm motor and upper arm rotation. */
            public static final double UPPER_ARM = UPPER_ARM_MOTOR * UPPER_ARM_CHAIN;
        }
        /** TODO: More accurate measurements should be pulled from the robot CAD.
         * Arm kinematics can use **ANY UNIT**, as long as all units are
         * consistent. If inches are easier to use, change any arm-related
         * constant using metres to inches. (Don't forget 
         * EXTENSION_POSITION_CONVERSION).
         */
        public static final double EXTENSION_PULLEY_DIAMETER = 0.055; // metres
        public static final double EXTENSION_PULLEY_CIRCUMFERENCE = Math.PI * EXTENSION_PULLEY_DIAMETER; // metres
        /** The distance from the centre of the tube connecting the lower arm
         * to the robot to the centre of the tube connecting the upper arm to
         * the lower arm.
         */
        public static final double LOWER_ARM_LENGTH = 0.51; // metres
        /** The distance from the centre of the tube connecting both arms to
         * the beginning of the gripper opening with the extension retracted. */
        public static final double UPPER_ARM_BASE_LENGTH = 0.67; // metres
        /** How fast to extend the arm extension during manual control.
         * Range is between -1 to 1, however, should be >= 0.
         */
        public static final double MANUAL_EXTENSION_SPEED = 0.1;
    }
    /** Autonomous-Related Constants */
    public static final class Autonomous {
        /** The NAME of the path, excluding its filepath and extension.
         * Path assumed to be `src/main/deploy/pathplanner/`.
         * Extension assumed to be `.path`.
        */
        public static final String PATH_NAME = "Triangle";
        /** Which port PathPlannerServer should connect to on the RoboRIO. */
        public static final int PATHPLANNER_SERVER_PORT = 5811;
        /** Entries in this map must be non-null, or the program will crash. */
        public static final HashMap<String, Command> eventMap = new HashMap<>(
            Map.ofEntries(
                entry("MarkerName", new TimerDelay(5)) // markerName, Command
            )
        );
        /** X and Y PID constants for path following. 
         * Setting these to 0 will use only feedforward.
         */
        public static final PIDConstants translationPIDConstants = new PIDConstants(1, 0, 0);
        /** Rotation PID constants for path following.
         * Setting these to 0 will use only feedforward.
        */
        public static final PIDConstants rotationPIDConstants = new PIDConstants(1, 0, 0);
    }
    /** IDs of all CAN bus devices. */
    public final class CANID {
        public static final int PNEUMATICS_HUB = 2;
        public static final int frontLeftDrive = 6;
        public static final int frontLeftRotation = 5;
        public static final int frontRightDrive = 9;
        public static final int frontRightRotation = 8;
        public static final int rearLeftDrive = 12;
        public static final int rearLeftRotation = 11;
        public static final int rearRightDrive = 15;
        public static final int rearRightRotation = 14;
        public static final int LOWER_ARM_MOTOR = 20;
        public static final int UPPER_ARM_MOTOR = 21;
        public static final int EXTENSION_MOTOR = 22;
    }
    public static final class DIO {
        public static final int ARM_LOWER_ENCODER = 0;
        public static final int ARM_UPPER_ENCODER = 1;
    }
    /** Swerve drive related constants. */
    public final class Swerve {
        /** Swerve drive PID and feedforward gains. 
         * setVoltage() is used to set motor power, as it ensures the motor
         * always outputs the same force when the battery voltage sags. 
         * Since setVoltage() is being used, these gains are tuned to produce
         * a *voltage* value, not a speed value, so `set()` should not be used
         * with any controller using these gains.
         */
        public final class Gains {
            public static final double drivePID_kP = 1;
            public static final double drivePID_kI = 0;
            public static final double drivePID_kD = 0;

            public static final double rotationPID_kP = 0.4;
            public static final double rotationPID_kI = 0.0001;
            public static final double rotationPID_kD = 1.0;
            public static final double rotationPID_IZone = 0.001;

            public static final double driveFF_kS = 0;
            public static final double driveFF_kV = 3.333;
            public static final double driveFF_kA = 0;

            // SparkMaxPIDController only has 1 feedforward constant.
            public static final double rotationFF = 0;
        }
        /** Constants related to swerve module dimensions. */
        public final class Dimensions {
            /* Direction Polarity:
                      +X (Front)
                 FL       ^       FR
                          |
            +Y (Left) <-------> -Y (Right)
                          |
                 RL       v       RR
                      -X (Rear)
             * Direction  | X | Y |
             * Front Left   +   +
             * Front Right  +   -
             * Rear Left    -   +
             * Rear Right   -   -
             */
            /** Front-rear (X) distance between two swerve modules measured from centre of wheel in metres. */
            public static final double TRACK_SIZE_X = 0.6096; // 24 inches (in metres)
            /** Left-right (Y) distance between two swerve modules measured from centre of wheel in metres. */
            public static final double TRACK_SIZE_Y = 0.6096; // 24 inches (in metres)
            /** Front-rear (X) distance from centre of wheel to centre of robot in metres. */
            public static final double X_FROM_CENTRE = TRACK_SIZE_X / 2;
            /** Left-right (Y) distance from centre of wheel to centre of robot in metres. */
            public static final double Y_FROM_CENTRE = TRACK_SIZE_Y / 2;
            public static final double frontLeftX   = X_FROM_CENTRE;
            public static final double frontLeftY   = Y_FROM_CENTRE;
            public static final double frontRightX  = X_FROM_CENTRE;
            public static final double frontRightY  = -Y_FROM_CENTRE;
            public static final double rearLeftX    = -X_FROM_CENTRE;
            public static final double rearLeftY    = Y_FROM_CENTRE;
            public static final double rearRightX   = -X_FROM_CENTRE;
            public static final double rearRightY   = -Y_FROM_CENTRE;
            /** Diagonal distance between opposite swerve modules.
             * Calculation: Math.sqrt(TRACK_SIZE_X * TRACK_SIZE_X + TRACK_SIZE_Y * TRACK_SIZE_Y)
            */
            public static final double DIAMETER = 0.862104588;
            /** Distance travelled when spinning in a circle. */
            public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
        }
        /** Physical acceleration and velocity limits. */
        public final class PhysicalLimits {
            /** The maximum possible RPM of a REV NEO v1.0/v1.1 motor. */
            public static final double MAX_NEO_RPM = 5676;
            /** The maximum angular acceleration the robot can achieve in radians/s^2. */
            // public static final double MAX_POSSIBLE_ANGULAR_ACCELERATION = 2 * Math.PI; // unused
            /** The maximum linear speed a swerve module can achieve in m/s. */
            public static final double MAX_POSSIBLE_LINEAR_SPEED = MAX_NEO_RPM * DRIVE_VELOCITY_CONVERSION;
            /** The maximum speed the robot can spin in radians/s. */
            public static final double MAX_POSSIBLE_ANGULAR_VELOCITY = 2 * Math.PI * (MAX_POSSIBLE_LINEAR_SPEED / Dimensions.CIRCUMFERENCE);
        }
        /** Autonomous acceleration and velocity limits.
         * Should match limits specified in PathPlanner.
         * This may not be a great place to put these constants as they are
         * path-specific, but are currently applied to all followed paths.
         */
        public final class AutonomousLimits {
            /** The maximum linear acceleration the robot should achieve in m/s^2. */
            public static final double MAX_LINEAR_ACCELERATION = 1.0; // TODO: adjust value
            /** The maximum speed the drivetrain should go in autonomous in m/s. */
            public static final double MAX_LINEAR_VELOCITY = 2.0;
        }
        /** Teleop acceleration and velocity limits */
        public final class TeleopLimits {
            /** The maximum speed the robot should spin in teleop in radians/s. */
            public static final double MAX_ANGULAR_VELOCITY = PhysicalLimits.MAX_POSSIBLE_ANGULAR_VELOCITY;
            /** The maximum speed the drivetrain should go in teleop in m/s. */
            public static final double MAX_LINEAR_VELOCITY = PhysicalLimits.MAX_POSSIBLE_LINEAR_SPEED;
        }
        // Miscellaneous:
        public static final double DEBUG_DRIVE_ANGLE_SENSITIVITY = 0.25;
        /** The maximum angular acceleration the robot can achieve in radians/s^2. */
        // public static final double MAX_POSSIBLE_ANGULAR_ACCELERATION = 2 * Math.PI; // unused
        /** The maximum speed the robot can spin in radians/s. */
        // public static final double MAX_POSSIBLE_ANGULAR_VELOCITY = 4 * Math.PI; // unused
        /** The maximum linear speed a swerve module can achieve in m/s. */
        public static final double MAX_POSSIBLE_LINEAR_VELOCITY = 3.8;
        /** Any speeds below this value will not cause the module to move. */
        public static final double MODULE_SPEED_DEADBAND = 0.001; // m/s
        /** The amount of module rotations for each rotation motor rotation.
         * <p>The motor is connected to a 5:1 gearbox and a 4:1 gearbox, and
         * the module spins once for every 3 rotations of the UltraPlanetary's
         * output.
         */
        /* May want to verify this gear ratio, as this math should be
         * equivalent to before, but has a different value.
         */
        public static final double ROTATION_GEAR_RATIO = 3.0 * UltraPlanetaryRatios.FIVE_TO_ONE * UltraPlanetaryRatios.FOUR_TO_ONE;
        public static final double WHEEL_RADIUS = 0.0508; // metres
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // metres
        /** 8.0 rotations of motor = 1.0 rotation of wheel */
        public static final double WHEEL_GEAR_RATIO = 1.0 / 8.0; // .0 to avoid integer division
        // Limiters:
        public static final double X_SPEED_DELAY = 3; // 1/x seconds from 0 -> 1
        public static final double Y_SPEED_DELAY = 3; // 1/x seconds from 0 -> 1
        public static final double ROTATION_DELAY = 3; // 1/x seconds from 0 -> 1
        /* Conversions:
         * Spark Max encoders return rotations for getPosition() and rotations/minute for getVelocity().
         * Their return value is automatically multiplied by their conversion factor.
         */
        /** Converts to distance travelled in metres.
         * Explanation:
         * Spark Max outputs total rotations.
         * Total rotations * circumference of wheel * wheel gear ratio = distance travelled in metres.
         */
        public static final double DRIVE_POSITION_CONVERSION = WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        /** Converts to metres per second.
         * Explanation:
         * Spark Max outputs RPM (rotations per minute).
         * RPM * motor : wheel gear ratio = wheel RPM
         * Wheel RPM / 60 seconds/minute = wheel RPS
         * Wheel RPS * wheel circumference in metres = m/s.
         * Note: Actual equation is equivalent to procedure above, but simplified.
         */
        public static final double DRIVE_VELOCITY_CONVERSION = (WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE) / 60;
        /** Converts to module position in radians.
         * Explanation:
         * Spark Max outputs total rotations.
         * Total rotations * rotation gear ratio = module rotations (position).
         * Module rotations * (2 * PI) radians/rotation = rotations in radians (position).
         */
        public static final double ROTATION_POSITION_CONVERSION = 2 * Math.PI * ROTATION_GEAR_RATIO;
        /** Converts to metres per second.
         * Explanation:
         * Spark Max outputs RPM (rotations per minute).
         * RPM * motor : rotation gear ratio = module RPM
         * Module RPM / 60 seconds/minute = module RPS
         * Module RPS * 2 * Pi radians/rotation = radians/second.
         * Note: Actual equation is equivalent to procedure above, but simplified.
         */
        public static final double ROTATION_VELOCITY_CONVERSION = (2 * Math.PI * ROTATION_GEAR_RATIO) / 60;
    }
    /**
     * UltraPlanetary gearbox ratios differ from the ratio printed on
     * the gearbox's side. This class contains the actual ratios of the
     * UltraPlanetary cartridges.
     */
    public static final class UltraPlanetaryRatios {
        /** The ratio of a 3:1 UltraPlanetary cartridge used as a reduction. */
        public static final double THREE_TO_ONE = 1.0 / (84.0 / 29.0);
        /** The ratio of a 4:1 UltraPlanetary cartridge used as a reduction. */
        public static final double FOUR_TO_ONE = 1.0 / (76.0 / 21.0);
        /** The ratio of a 5:1 UltraPlanetary cartridge used as a reduction. */
        public static final double FIVE_TO_ONE = 1.0 / (68.0 / 13.0);
    }

    /** 
     * Xbox controller related constants. 
     * Do not put button or axis numbers in here, instead use the functions it
     * provides, such as getLeftY() or getXButton().
     */
    public final class Xbox {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double JOYSTICK_DEADBAND = 0.075;
    }
}
