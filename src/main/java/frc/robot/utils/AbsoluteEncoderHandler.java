// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Makes absolute encoders a bit easier to use. */
public class AbsoluteEncoderHandler {
    private final DutyCycleEncoder encoder;
    private final double positionOffset;
    private final double positionConversionFactor;

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
    }

    public double getRawValue() {
        return encoder.getAbsolutePosition();
    }

    public double getPosition() {
        return encoder.getDistance() - (positionOffset * positionConversionFactor);
    }
}
