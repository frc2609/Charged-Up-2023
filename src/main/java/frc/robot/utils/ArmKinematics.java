// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class ArmKinematics {
    // Link lengths and joint angles
    private static final double l1 = 0.7; // length of lower link
    private static final double l2 = 0.7; // length of upper link

    // Link masses and CoM positions from base
    private static final double m1 = 1.448;
    private static final double r1 = -0.6536; // center of mass of lower link
    private static final double m2 = 1.31096512;
    private static final double r2 = -0.254; // center of mass of upper link
    private static final double m3 = 1.245416864;
    private static final double r3 = -0.2413; // center of mass of extension link

    // Gravitational acceleration
    private static final double g = 9.81;

    public double[] forwardKinematics(double theta1, double theta2, double d3) {
        double x = l1 * Math.cos(theta1) + l2 * Math.cos(theta1 + theta2);
        double y = l1 * Math.sin(theta1) + l2 * Math.sin(theta1 + theta2) + d3;
        return new double[]{x, y};
    }

    public static double[] gravitationalTorques(double lower, double upper, double ext) {
        double[] thetas = robotAngleToArray(lower, upper, ext);
        double theta1 = thetas[0];
        double theta2 = thetas[1];
        double d3 = thetas[2];
        // Z-coordinates of each link's CoM
        double z1 = l1 * Math.sin(theta1) + r1;
        double z2 = z1 + l2 * Math.sin(theta1 + theta2) + r2;
        double z3 = z2 + d3 + r3;

        // Gravitational torques
        double tau1 = -g * (m1 * z1 + m2 * (z1 + z2) + m3 * (z1 + z2 + z3));
        double tau2 = -g * (m2 * z2 + m3 * (z2 + z3));
        double tau3 = -g * m3 * z3;

        return new double[]{tau1, tau2, tau3};
    }
    
    public static double[] robotAngleToArray(double theta1, double theta2, double theta3) {
        return new double[]{
            Math.toRadians(theta1),
            Math.toRadians(180 - theta2),
            metersToInches(theta3)
        };
    }

    public static double[] arrayToRobotAngle(double[] arr) {
        if (arr.length != 3) {
            throw new IllegalArgumentException("Array should have a length of 3");
        }
        
        return new double[]{
            Math.toDegrees(arr[0]),
            Math.toDegrees(Math.toRadians(180) - arr[1]),
            inchesToMeters(arr[2])
        };
    }
    
    public static double inchesToMeters(double inches) {
        return inches / 39.3701;
    }
    
    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

}
