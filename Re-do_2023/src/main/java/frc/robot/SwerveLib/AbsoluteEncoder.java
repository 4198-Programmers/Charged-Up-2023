package frc.robot.SwerveLib;

import com.ctre.phoenix.ErrorCode;

public interface AbsoluteEncoder {
    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    double getAbsoluteAngle();

    ErrorCode sendLastEncoderErr();
}
