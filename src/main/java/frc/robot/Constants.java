// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

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
            /** How many degrees the arm moves per motor rotation. */
            public static final double LOWER_POSITION_CONVERSION = Ratios.LOWER_ARM * 360.0;
            /** How many degrees the arm moves per motor rotation. */
            public static final double UPPER_POSITION_CONVERSION = Ratios.UPPER_ARM * 360.0;
            /** Pointing straight up (angle = 90.0 degrees). 
             * Encoder values increase as the arm moves away from the front of
             * the robot.
             */
            public static final double LOWER_POSITION_OFFSET = 0.625;
            /** Parallel robot front (angle = 90.0 degrees). 
             * Encoder values increase as the arm moves away from the front of
             * the robot.
             */
            public static final double UPPER_POSITION_OFFSET = 0.814;
        }
        public static final class Pneumatics {
            public static final int OPEN_SOLENOID_ID = 14;
            public static final int CLOSE_SOLENOID_ID = 15;
        }
        public static final class Position {
            public static final double EXIT_STOW_LOWER = 104.5;
            public static final double EXIT_STOW_UPPER = 38.3;
            public static final double EXIT_STOW_EXTENSION = 0.0;
            public static final double GROUND_PICKUP_LOWER = 72.0;
            public static final double GROUND_PICKUP_UPPER = 32.4;
            public static final double GROUND_PICKUP_EXTENSION = 0.0;
            public static final double LOW_LOWER = 77.6;
            public static final double LOW_UPPER = 49.6;
            public static final double LOW_EXTENSION = 0.0;
            public static final double MID_LOWER = 62.0;
            public static final double MID_UPPER = 139.7;
            public static final double MID_EXTENSION = 0.0;
            public static final double HIGH_LOWER = 58.5;
            public static final double HIGH_UPPER = 156.0;
            public static final double HIGH_EXTENSION = 0.440;
            public static final double PICKUP_LOWER = 98.9;
            public static final double PICKUP_UPPER = 91.3;
            public static final double PICKUP_EXTENSION = 0.07;
            public static final double RETRACT_LOWER = 110.0;
            public static final double RETRACT_UPPER = 94.0;
            public static final double RETRACT_EXTENSION = 0.0;
            public static final double STOW_LOWER = 104.60;
            public static final double STOW_UPPER = 19;
            public static final double STOW_EXTENSION = 0.0;
        }
        public static final class Ratios {
            /** Extension pulley rotations per extension motor rotation. */
            public static final double EXTENSION_MOTOR = UltraPlanetaryRatios.FIVE_TO_ONE * UltraPlanetaryRatios.FOUR_TO_ONE;
            /** Ratio between shaft pulley (18t) and arm pulley (48t). */
            public static final double LOWER_ARM_CHAIN = 18.0 / 48.0;
            /** Arm shaft rotations per motor rotation. (45:1) */
            public static final double LOWER_ARM_MOTOR = 1.0 / 45.0;
            /** Total ratio between lower arm motor and lower arm rotation. */
            public static final double LOWER_ARM = LOWER_ARM_MOTOR * LOWER_ARM_CHAIN;
            /** Ratio between shaft pulley (18t) and arm pulley (48t). */
            public static final double UPPER_ARM_CHAIN = 18.0 / 48.0;
            /** Arm shaft rotations per motor rotation. (2x 5:1 + 1x 3:1) */
            public static final double UPPER_ARM_MOTOR = 1.0 / 45.0;
            /** Total ratio between upper arm motor and upper arm rotation. */
            public static final double UPPER_ARM = UPPER_ARM_MOTOR * UPPER_ARM_CHAIN;
        }
        public static final class SoftStop {
            // `f` indicates a float literal
            public static final float LOWER_FORWARD = 148.70f;  // degrees
            public static final float LOWER_REVERSE = 27.45f;   // degrees
            public static final float UPPER_FORWARD = 177.00f;  // degrees
            public static final float UPPER_REVERSE = 10.75f;   // degrees
            public static final float EXTENSION_FORWARD = 0.48f; // metres
            public static final float EXTENSION_REVERSE = 0.0f;  // metres
        }
        public static final class Tolerances {
            public static final double LOWER_ANGLE = 0.5; // degrees
            public static final double UPPER_ANGLE = 0.5; // degrees
            public static final double EXTENSION_LENGTH = 0.01; // metres
            // TODO: is it possible to increase the EXTENSION_LENGTH TOLERANCE?
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
        public static final double MANUAL_UPPER_INCREMENT = 2.5;
        public static final double MANUAL_EXTENSION_INCREMENT = 0.04;
        /** Change in setpoint in degrees per second. */
        public static final double MANUAL_LOWER_ACCELERATION = 80.0;
        /** Change in setpoint in degrees per second. */
        public static final double MANUAL_UPPER_ACCELERATION = 80.0;
        /** Change in setpoint in metres per second. */
        public static final double MANUAL_EXTENSION_ACCELERATION = 40.0 / 100.0;
    }
    /** Autonomous-Related Constants */
    public static final class Autonomous {
        /** Autobalance-related constants. */
        public static final class Balance {
            /** P gain for going up the ramp on the charge station. */
            public static final double START_P = 0.05;
            /** P gain for staying on the charge station. */
            public static final double HOLD_P = 0.025;
            /** The acceptable amount of tilt error in degrees. */
            public static final double ANGLE_TOLERANCE = 5;
            /** The maximum autobalance speed in metres per second. */
            public static final double MAX_SPEED = 1.5;
        }
        /** Deadlines (in seconds) to wait for a command to finish. */
        public static final class Deadline {
            public static final double MOVE_TO_HIGH = 5.0;
        }
        /** Which port PathPlannerServer should connect to on the RoboRIO. */
        public static final int PATHPLANNER_SERVER_PORT = 5811;
        /** X and Y PID constants for path following. 
         * Setting these to 0 will use only feedforward.
         */
        public static final PIDConstants translationPIDConstants = new PIDConstants(0, 0, 0);
        /** Rotation PID constants for path following.
         * Setting these to 0 will use only feedforward.
        */
        public static final PIDConstants rotationPIDConstants = new PIDConstants(0, 0, 0);
    }
    /** IDs of all CAN bus devices. */
    public final static class CANID {
        public static final int PNEUMATICS_HUB = 2;
        // TODO: rename
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
        public static final int LOWER_ARM_MOTOR = 20;
        public static final int UPPER_ARM_MOTOR = 21;
        public static final int EXTENSION_MOTOR = 22;
    }
    public static final class DIO {
        public static final int ARM_LOWER_ENCODER = 0;
        public static final int ARM_UPPER_ENCODER = 1;
        public static final int GRIPPER_SENSOR = 7;
    } 
    public static final class LED {
        public static final double BLUE = 0.87;
        public static final double GREEN = 0.77;
        public static final double LIME = 0.73;
        public static final double RED = 0.61;
        public static final double VIOLET = 0.91;
        public static final double YELLOW = 0.69;
        public static final double C1_STROBE = 0.15;
        public static final double C2_STROBE = 0.35;
        public static final double BREATH_RED = -0.17;
        public static final double BREATH_BLUE = -0.15;
    }
    /** Smart current limits for Spark Max motor controllers in amps. */
    public static final class Limits {
        public static final int DRIVE_PRIMARY_CURRENT = 50;
        public static final int DRIVE_SECONDARY_CURRENT = 50;
        public static final int DRIVE_ROTATION_CURRENT = 30;
        public static final int LOWER_ARM_CURRENT = 40;
        public static final int UPPER_ARM_CURRENT = 40;
        public static final int EXTENSION_CURRENT = 40;
    }
    public static final class PWMID {
        public static final int REV_BLINKIN = 0;
    }
    /** Swerve drive related constants. */
    public final static class Swerve {
        /** Swerve drive PID and feedforward gains. 
         * setVoltage() is used to set motor power, as it ensures the motor
         * always outputs the same force when the battery voltage sags. 
         * Since setVoltage() is being used, these gains are tuned to produce
         * a *voltage* value, not a speed value, so `set()` should not be used
         * with any controller using these gains.
         */
        public final static class Gains {
            public static final double drivePID_kP = 0.001;
            public static final double drivePID_kI = 0;
            public static final double drivePID_kD = 0;

            
            public static final double drivePID_kP_auto = 0.001;
            public static final double drivePID_kI_auto = 0;
            public static final double drivePID_kD_auto = 0;
            public static final double drivePID_kF_auto = 0.3;

            
            public static final double rotationPID_kP_auto = 1;
            public static final double rotationPID_kI_auto = 0.0001;
            public static final double rotationPID_kD_auto = 1.0;
            public static final double rotationPID_IZone_auto = 0.001;

            public static final double rotationPID_kP = 0.4;
            public static final double rotationPID_kI = 0.0001;
            public static final double rotationPID_kD = 1.0;
            public static final double rotationPID_IZone = 0.001;

            public static final double driveFF_kS = 0;
            public static final double driveFF_kV = 3.333;
            public static final double driveFF_kA = 0;

            
            public static final double driveFF_kS_auto = 0;
            public static final double driveFF_kV_auto = 2.609;
            public static final double driveFF_kA_auto = 0;

            // SparkMaxPIDController only has 1 feedforward constant.
            public static final double rotationFF = 0;
            public static final double rotationFF_auto = 0.07;
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
            public static final double MAX_LINEAR_ACCELERATION = 1.5;
            /** The maximum speed the drivetrain should go in autonomous in m/s. */
            public static final double MAX_LINEAR_VELOCITY = 1.5;
        }
        /** Teleop acceleration and velocity limits */
        public final static class TeleopLimits {
            /** The maximum speed the robot should spin in teleop in radians/s. */
            public static final double MAX_ANGULAR_VELOCITY = PhysicalLimits.MAX_POSSIBLE_ANGULAR_VELOCITY;
            /** The maximum speed the drivetrain should go in teleop in m/s. */
            public static final double MAX_LINEAR_VELOCITY = PhysicalLimits.MAX_POSSIBLE_LINEAR_SPEED;
        }
        // Miscellaneous:
        /** 
         * The allowed rotation error in radians when rotating a module to a
         * specific angle (not used when setting a SwerveModuleState).
         */
        public static final double ROTATION_ANGLE_TOLERANCE = 0.1;
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
        // TODO: fix this math
        public static final double ROTATION_GEAR_RATIO = 1.0 / 56.6409;//3.0 * UltraPlanetaryRatios.FIVE_TO_ONE * UltraPlanetaryRatios.FOUR_TO_ONE;//1.0 / 56.6409; 
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
    public final static class Xbox {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double JOYSTICK_DEADBAND = 0.075;
    }
}
