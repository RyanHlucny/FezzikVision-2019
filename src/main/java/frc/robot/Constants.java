package frc.robot;

import java.lang.Math;

/**
 * This class stores parameters used across the entire robot. They include physical robot parameters,
 * mathematical constants, and important/useful device constants.
 */
public class Constants {

    /* TUNABLE PARAMETERS */

    // Drive parameters
    
    // Auto-shifting
    // Speed threshold for auto-shifting up in ft/s
    public static final double AUTO_SHIFT_UP_SP_THRESH = 8.0;
    // Speed threshold for auto-shifting down in ft/s
    public static final double AUTO_SHIFT_DOWN_SP_THRESH = 6.0;
    // Angular rate threshold for auto-shifting down in radians/s
    public static final double AUTO_SHIFT_DOWN_ANGULAR_THRESH = 7.53;
    
    // Tuned dynamics for path following
    public static final double ROBOT_LINEAR_INERTIA = 60.0; // kg
    public static final double ROBOT_ANGULAR_INERTIA = 10.0; // kg m^2
    public static final double ROBOT_ANGULAR_DRAG = 12.0; // N*m / (rad/sec)
    public static final double DRIVE_V_INTERCEPT = 1.055; // V
    public static final double DRIVE_Kv = 1.0 / 0.3; // rad/s per V
    public static final double DRIVE_Ka = 1.0 / 0.39; // rad/s^2 per V

    // PID gains for drive velocity loop (low gear)
    public static final double LOW_GEAR_VELOCITY_Kp = 0.9; // 0.9
    public static final double LOW_GEAR_VELOCITY_Ki = 0.0;
    public static final double LOW_GEAR_VELOCITY_Kd = 5.0; // 10.0
    public static final double LOW_GEAR_VELOCITY_Kf = 0.0;
    public static final int LOW_GEAR_VELOCITY_IZONE = 0;

    // PID gains for drive velocity loop (high gear)
    public static final double HIGH_GEAR_VELOCITY_Kp = 0.9;
    public static final double HIGH_GEAR_VELOCITY_Ki = 0.0;
    public static final double HIGH_GEAR_VELOCITY_Kd = 5.0;
    public static final double HIGH_GEAR_VELOCITY_Kf = 0.0;
    public static final int HIGH_GEAR_VELOCITY_IZONE = 0;

    // PID gains for drive auto-steer (low gear)
    public static final double AUTO_STEER_Kp = 0.02;

    /* ROBOT PHYSICAL CONSTANTS */
    
    // Wheels
    public static final double WHEEL_BASE_INCHES = 25.5;
    public static final double WHEEL_DIAMETER_INCHES = 6.0;
    public static final double WHEEL_RADIUS_INCHES = WHEEL_DIAMETER_INCHES / 2.0;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double WHEEL_SCRUB_FACTOR = 1.0;

    // Chassis dimensions
    public static final double ROBOT_FRAME_WIDTH = 27.5; // inches
    public static final double ROBOT_FRAME_LENGTH = 32.0;
    public static final double ROBOT_FRAME_WIDTH_PLUS_BUMPERS = 34.0;
    public static final double ROBOT_FRAME_LENGTH_PLUS_BUMPERS = 38.5;

    /* ELECTRONIC CONSTANTS */

    // Drive encoders
    // Encoder ticks per revolution for the drive wheels (assuming 4X decoding)
    public static final double DRIVE_TICKS_PER_REV = 800;

    /* I/O */

    // CAN
    public static final int CAN_TIMEOUT_MS = 10; // Use for updates
    public static final int LONG_CAN_TIMEOUT_MS = 100; // use for constructors
}