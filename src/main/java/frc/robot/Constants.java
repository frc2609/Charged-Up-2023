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
    /** Swerve drive related constants. */
    public final static class Swerve {
        /** CAN IDs of swerve module motor controllers. */
        public final static class CANID {
            public static final int frontLeftPrimary = 6;
            public static final int frontLeftSecondary = 7;
            public static final int frontLeftRotation = 5;
            public static final int frontRightPrimary = 9;
            public static final int frontRightSecondary = 10;
            public static final int frontRightRotation = 8;
            public static final int rearLeftPrimary = 12;
            public static final int rearLeftSecondary = 13;
            public static final int rearLeftRotation = 11;
            public static final int rearRightPrimary = 15;
            public static final int rearRightSecondary = 16;
            public static final int rearRightRotation = 14;
        }
        /** Swerve drive PID and feedforward gains. 
         * setVoltage() is used to set motor power, as it ensures the motor
         * always outputs the same force when the battery voltage sags. 
         * Since setVoltage() is being used, these gains are tuned to produce
         * a *voltage* value, not a speed value, so `set()` should not be used
         * with any controller using these gains.
         */
        public final static class Gains {
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
        public final static class IsInverted {
            public static final boolean frontLeftDrive = true;
            public static final boolean frontRightDrive = false;
            public static final boolean rearLeftDrive = false;
            public static final boolean rearRightDrive = true;
            public static final boolean frontLeftRotation = true;
            public static final boolean frontRightRotation = true;
            public static final boolean rearLeftRotation = true;
            public static final boolean rearRightRotation = true;
        }
        /** Constants related to swerve module dimensions. */
        public final static class Dimensions {
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
        public final static class PhysicalLimits {
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
        public final static class AutonomousLimits {
            /** The maximum linear acceleration the robot should achieve in m/s^2. */
            public static final double MAX_LINEAR_ACCELERATION = 1.0; // TODO: adjust value
            /** The maximum speed the drivetrain should go in autonomous in m/s. */
            public static final double MAX_LINEAR_VELOCITY = 2.0;
        }
        /** Teleop acceleration and velocity limits */
        public final static class TeleopLimits {
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
        /** 56.6409 rotations of motor = 1.0 rotation of module 
         * <p>UltraPlanetary gearbox ratios differ from the ratio printed on
         * the gearbox's side. The motor is connected to a 5:1 gearbox and a
         * 4:1 gearbox, whose actual ratios are 5.23:1 and 3.61:1 respectively.
         * The module spins once for every 3 rotations of the UltraPlanetary's
         * output, which gives a gear ratio of 5.23 x 3.61 x 3 = 56.6409.
         */
        public static final double ROTATION_GEAR_RATIO = 1.0 / 56.6409; // .0 to avoid integer division
        public static final double WHEEL_RADIUS = 0.0508; // metres
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // metres
        /** 1.0 rotations of motor = 3.0 rotation of the carrier */
        public static final double WHEEL_GEAR_RATIO = 1.0 / 3.0;
        // Limiters:
        public static final double X_SPEED_DELAY = 3; // 1/x seconds from 0 -> 1
        public static final double Y_SPEED_DELAY = 3; // 1/x seconds from 0 -> 1
        public static final double ROTATION_DELAY = 6; // 1/x seconds from 0 -> 1
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
     * Xbox controller related constants. 
     * Do not put button or axis numbers in here, instead use the functions it
     * provides, such as getLeftY() or getXButton().
     */
    public final static class Xbox {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double JOYSTICK_DEADBAND = 0.125;
    }

    public final class LED {
        public static final int PWM_PORT = 0;
        public static final double GREEN = 0.77;
        public static final double RED = 0.61;
        public static final double VIOLET = 0.91;
        public static final double YELLOW = 0.69;
        public static final double BLUE = 0.87;
        public static final double BREATH_RED = -0.17;
        public static final double BREATH_BLUE = -0.15;
    }
}
