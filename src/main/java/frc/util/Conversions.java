package frc.util;

/**
 * This class contains useful methods for converting between units.
 */
public class Conversions {
    
    /**
     * Converts encoder velocity to linear velocity, assuming that the encoder is measuring the angular velocity of a wheel.
     * This is useful when getting linear velocity measurements from a robot's drivetrain.
     * @param encoderVelocity The encoder velocity to convert in pulses/sec (If using native TalonSRX units, multiply by 10)
     * @param encoderTicksPerRev Amount of encoder pulses (ticks) per revolution
     * @param wheelCirc The circumference of the wheel(s) in inches
     * @return Linear velocity in ft/s
     */
    public static double encoderVelocityToLinearVelocity(double encoderVelocity, double encoderTicksPerRev, double wheelCirc) {
        return encoderVelocity / encoderTicksPerRev * wheelCirc / 12.0;
    }

    /**
     * Convert angular velocity to encoder velocity.
     * @param angularVelocity The angular velocity to convert in rad/s
     * @param encoderTicksPerRev Amount of encoder pulses (ticks) per revolution
     * @return Encoder velocity in ticks/s (divide by 10 for native TalonSRX units)
     */
    public static double angularVelocityToEncoderVelocity(double angularVelocity, double encoderTicksPerRev) {
        return angularVelocity / (Math.PI * 2.0) * encoderTicksPerRev;
    }

    /**
     * Converts encoder ticks to wheel rotations, assuming that the encoder is measuring the angular position of a wheel.
     * This is useful when getting the linear position of a robot.
     * @param ticks The encoder ticks to convert to rotations
     * @param encoderTicksPerRev Amount of encoder pulses (ticks) per revolution
     * @return number of rotations
     */
    public static double encoderTicksToRotations(double ticks, double encoderTicksPerRev) {
        return ticks / encoderTicksPerRev;
    }

    /**
     * Converts encoder ticks to linear distance in inches, assuming that the encoder is measuring the angular position of a wheel.
     * This is useful when getting the linear position of a robot.
     * @param ticks The encoder ticks to convert to inches
     * @param encoderTicksPerRev Amount of encoder pulses (ticks) per revolution
     * @param wheelCirc The circumference of the wheel(s) in inches
     * @return Linear position in inches
     */
    public static double encoderTicksToInches(double ticks, double encoderTicksPerRev, double wheelCirc) {
        return encoderTicksToRotations(ticks, encoderTicksPerRev) * wheelCirc;
    }

    /**
     * Converts encoder ticks to linear distance in feet, assuming that the encoder is measuring the angular position of a wheel.
     * This is useful when getting the linear position of a robot.
     * @param ticks The encoder ticks to convert to feet
     * @param encoderTicksPerRev Amount of encoder pulses (ticks) per revolution
     * @param wheelCirc The circumference of the wheel(s) in inches
     * @return Linear position in feet
     */
    public static double encoderTicksToFeet(double ticks, double encoderTicksPerRev, double wheelCirc) {
        return encoderTicksToInches(ticks, encoderTicksPerRev, wheelCirc) / 12.0;
    }
    
}