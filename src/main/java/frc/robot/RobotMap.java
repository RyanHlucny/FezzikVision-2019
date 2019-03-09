package frc.robot;

/**
 * The RobotMap provides a mapping between physical ports (for sensors, actuators, and other devices)
 * and logical variable names. This makes the code easier to understand by reducing the amount of magic numbers.
 */
public class RobotMap {
  //PDP port number
	public static final int PDP = 0;
	
	// **Drive Subsystem**
	
	// Drive motors
	public static final int FRONT_LEFT_MOTOR = 3;
	public static final int REAR_LEFT_MOTOR = 4;
	public static final int FRONT_RIGHT_MOTOR = 5;
	public static final int REAR_RIGHT_MOTOR = 6;
	
	// shifting pneumatics
	public static final int SHIFT_LOW_CHANNEL = 0;
	public static final int SHIFT_HIGH_CHANNEL = 1;
	
	
	// **Elevator Subsystem**
	public static final int ELEVATOR_MASTER = 1;
	public static final int ELEVATOR_SLAVE = 7;
	
	
	// **Intake Subsystem**
	
	// Intake Motors
	public static final int INTAKE_LEFT = 3;
	public static final int INTAKE_RIGHT = 2;
	
	// PDP ports
	public static final int INTAKE_LEFT_PDP = 11;
	public static final int INTAKE_RIGHT_PDP = 10;
	
	// Intake pneumatics
	public static final int INTAKE_OPEN_CHANNEL = 2;
	public static final int INTAKE_CLOSE_CHANNEL = 3;
	
	// Intake sensors
	public static final int INTAKE_IR_ANALOG = 0;
	
	
	// **Header Subsystem**
	
	// Header motor
	public static final int HEADER_MOTOR = 0;
	
	
	// **Ejector Subsystem**
	
	// Pneumatics
	public static final int EJECTOR_FORWARD_CHANNEL = 0;
	public static final int EJECTOR_REVERSE_CHANNEL = 1;
	
	
	// **Winch Subsystem**
	
	// Winch Motors
	public static final int WINCH_1 = 0;
	public static final int WINCH_2 = 1;
	
	// Winch Pneumatics
	public static final int WINCH_FORWARD_CHANNEL = 2;
	public static final int WINCH_REVERSE_CHANNEL = 3;
	
	
	// **Wings Subsystem**
	
	// Wing Pneumatics
	public static final int WINGS_RIGHT_FORWARD_CHANNEL = 5;
	public static final int WINGS_RIGHT_REVERSE_CHANNEL = 4;
	public static final int WINGS_LEFT_FORWARD_CHANNEL = 7;
public static final int WINGS_LEFT_REVERSE_CHANNEL = 6;
}
