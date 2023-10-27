// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;

/** Makes absolute encoders a bit easier to use. */
public class AbsoluteEncoderHandler implements AutoCloseable {
    private static final ArrayList<AbsoluteEncoderHandler> absoluteEncoders = new ArrayList<AbsoluteEncoderHandler>();
    private final DutyCycleEncoder encoder;
    private final double positionOffset;
    private final double positionConversionFactor;
    private double previousPosition;
    private double previousTime;
    private double velocity = 0;

    /**
     * Create a new AbsoluteEncoderHandler
     * @param encoderDIOID The DIO port the encoder is plugged into.
     * @param positionOffset The absolute position of the encoder (0 - 1) when the mechanism the encoder is tracking is at a position of 0 units.
     * @param positionConversionFactor Multiplied by the current number of rotations to give position. Determines the units of 'getPosition()'.
     */
    public AbsoluteEncoderHandler(int encoderDIOID, double positionOffset, double positionConversionFactor) {
        encoder = new DutyCycleEncoder(encoderDIOID);
        this.positionOffset = positionOffset;
        this.positionConversionFactor = positionConversionFactor;
        encoder.setDistancePerRotation(positionConversionFactor);

        previousPosition = getPosition();
        previousTime = RobotController.getFPGATime();

        absoluteEncoders.add(this);
    }
    
    @Override
    public void close() {
        absoluteEncoders.remove(this);
    }

    /**
     * You should call this in robotPeriodic, or all velocities will be inaccurate!
     */
    public static void updateAllVelocities() {
        for (AbsoluteEncoderHandler absoluteEncoder : absoluteEncoders) {
            absoluteEncoder.updateVelocity();
        }
    }

    public double getRawValue() {
        return encoder.getAbsolutePosition();
    }

    public double getPosition() {
        return encoder.getDistance() - (positionOffset * positionConversionFactor);
    }

    /* This should be tested before use. */
    public double getVelocity() {
        return velocity;
    }

    protected void updateVelocity() {
        final double position = getPosition();
        final double time = RobotController.getFPGATime();
        velocity = (position - previousPosition) / (time - previousTime);
        previousPosition = position;
        previousTime = time;
    }
}
