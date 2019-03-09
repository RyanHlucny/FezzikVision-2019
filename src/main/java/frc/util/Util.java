package frc.util;

/**
 * This class contains useful functions that are used often.
 */
public class Util {

    /**
     * Prevents this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits a value to a specified magnitude.
     * @param val The value to limit
     * @param maxMagnitude The maximum magnitude to allow through
     * @return The limited input value
     */
    public static double limit(double val, double maxMagnitude) {
        return limit(val, -maxMagnitude, maxMagnitude);
    }

    /**
     * Limits a value to be within a range
     * @param val The value to limit
     * @param min The minimum value that can be returned
     * @param max The maximum value that can be returned
     * @return The input value limited within min and max
     */
    public static double limit(double val, double min, double max) {
        return Math.min(max, Math.max(min, val));
    }
}