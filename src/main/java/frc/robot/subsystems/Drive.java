package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.lib.team254.geometry.Pose2dWithCurvature;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.trajectory.TrajectoryIterator;
import frc.lib.team254.trajectory.timing.TimedState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.Conversions;
import frc.robot.RobotMap;
import frc.robot.commands.drive.OpenLoopDrive;
import frc.robot.planners.DriveMotionPlanner;
import frc.team5172.lib.util.Util;
import frc.team5172.lib.util.DriveSignal;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.lang.Math;

/**
 * Drive subsystem; controls the drivetrain of the robot.
 */
public class Drive extends Subsystem {
  // Constants
  private static final int kLowGearVelocityControlSlot = 0;
  private static final int kHighGearVelocityControlSlot = 1;
  private static final double kNeutralDeadband = 0.05; // Drive deadband when in open loop mode

  // Drive states
  public enum DriveControlMode { OPEN_LOOP, PATH_FOLLOWING, AUTO_STEER }
  public enum DriveGearState { LOW, HIGH }

  // Components
  private WPI_TalonSRX mfrontLeftCIM = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR);
	private WPI_TalonSRX mfrontRightCIM = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR);
	private WPI_TalonSRX mrearRightCIM = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR);
  private WPI_TalonSRX mrearLeftCIM = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR);
  private ADXRS450_Gyro mgyro = new ADXRS450_Gyro();
  private DoubleSolenoid mshiftSolenoid = new DoubleSolenoid(1, RobotMap.SHIFT_LOW_CHANNEL, RobotMap.SHIFT_HIGH_CHANNEL);

  // Systems
  private DriveMotionPlanner mMotionPlanner;

  // State variables
  private DriveControlMode currentControlMode = DriveControlMode.OPEN_LOOP;
  private DriveGearState currentGear = DriveGearState.LOW;
  private boolean brakeMode = false;
  private double gyroOffset = 0.0;
  private boolean followPathBackwards = false;
  // Variables for the auto-steer state machine
  private Rotation2d m_lastAngleSetpoint = null;

  /** Default constructor */
  public Drive() {
    // Add children to the subsystem
    addChild("Front Left CIM", mfrontLeftCIM);
    addChild("Front Right CIM", mfrontRightCIM);
    addChild("Rear Right CIM", mrearRightCIM);
    addChild("Rear Left CIM", mrearLeftCIM);
    addChild("Shifting Solenoid", mshiftSolenoid);
  
    // Set front motors to follow the rear motors (rear motors have encoders)
    mfrontLeftCIM.set(ControlMode.Follower, RobotMap.REAR_LEFT_MOTOR);
    mfrontRightCIM.set(ControlMode.Follower, RobotMap.REAR_RIGHT_MOTOR);
    mfrontRightCIM.setInverted(true);

    // Configure master talons
    configMasterTalon(mrearLeftCIM, true);
    configMasterTalon(mrearRightCIM, false);

    // Reload PID gains
    reloadGains();

    // Force a shifter message (initially shift into low gear)
    currentGear = DriveGearState.HIGH;
    shiftLow();

    // Force a brake message (initially set motors to coast)
    brakeMode = true;
    setBrakeMode(false);

    // Initialize motion planner
    mMotionPlanner = new DriveMotionPlanner();
  }

  /**
   * Configures a master CAN-based TalonSRX.
   * @param talon The talon to configure
   * @param isLeft Whether the talon is the master for the left side or not
   */
  private void configMasterTalon(WPI_TalonSRX talon, boolean isLeft) {
    final ErrorCode sensor_present = talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    if (sensor_present != ErrorCode.OK) {
      DriverStation.reportError("Could not detect " + (isLeft ? "left" : "right") + "encoder: " + sensor_present, false);
    }
    // Set the frame period for feedback0 status2 to 5 ms (default is 20ms)
    // This gives us faster encoder feedback updates
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    // Set safety timeout to 100 ms
    talon.setSafetyEnabled(true);
    talon.setExpiration(0.1);

    talon.setInverted(!isLeft);
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.LONG_CAN_TIMEOUT_MS);
    talon.configVelocityMeasurementWindow(1, Constants.LONG_CAN_TIMEOUT_MS);
    talon.configNeutralDeadband(kNeutralDeadband, Constants.LONG_CAN_TIMEOUT_MS);
  }

  /**
   * By default, the drivetrain should not move
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new OpenLoopDrive(DriveSignal.NEUTRAL));
  }

  /**
   * Arcade drive using a throttle and steering input
   * @param throttle open-loop throttle to set to the motors [-1.0, 1.0]
   * @param steering open-loop steering to use in the arcade drive [-1.0, 1.0]. Positive is CW
   * @param enableAutoShift Whether or not to enable auto shifting
   */
  public void arcadeDrive(double throttle, double steering, boolean enableAutoShift) {
    double left_power = throttle + steering;
    double right_power = throttle - steering;
    if (left_power > 1.0) {
      right_power -= (left_power - 1.0);
      left_power = 1.0;
    }
    else if (right_power < -1.0) {
      left_power += (-1.0 - right_power);
      right_power = -1.0;
    }
    else if (left_power < -1.0) {
      right_power += (-1.0 - left_power);
      left_power = -1.0;
    }
    else if (right_power > 1.0) {
      left_power -= (right_power - 1.0);
      right_power = 1.0;
    }

    driveOpenLoop(new DriveSignal(left_power, right_power), enableAutoShift);
  }

  /**
   * Controls the motors with an open loop drive signal
   * @param signal drive signal to set to the drivetrain motors
   */
  public void driveOpenLoop(DriveSignal signal, boolean enableAutoShift) {
    switchDriveControlModes(DriveControlMode.OPEN_LOOP);
    mrearLeftCIM.set(ControlMode.PercentOutput, signal.getLeft());
    mrearRightCIM.set(ControlMode.PercentOutput, signal.getRight());
    if (enableAutoShift) {
      handleAutoShifting();
    }
  }

  public void driveAutoSteer(DriveSignal signal, Rotation2d desiredAngle) {
    switchDriveControlModes(DriveControlMode.AUTO_STEER);
    Rotation2d angleSetpoint;
    // If a target hasn't been seen yet, just drive normally
    if (desiredAngle == null && m_lastAngleSetpoint == null) {
      mrearLeftCIM.set(ControlMode.PercentOutput, signal.getLeft());
      mrearRightCIM.set(ControlMode.PercentOutput, signal.getRight());
      setBrakeMode(signal.getBrakeMode());
      return;
    }
    // If target has been seen but is not visible currently, actuate based on last
    // known setpoint
    else if (desiredAngle == null && m_lastAngleSetpoint != null) {
      angleSetpoint = m_lastAngleSetpoint;
    }
    // Otherwise, use desiredAngle as the setpoint
    else {
      angleSetpoint = desiredAngle;
    }
    // Basic proportional control system
    // Compare angle setpoint with current heading
    Rotation2d angleError = getHeading().inverse().rotateBy(angleSetpoint);

    // Calculate control law
    double output = Constants.AUTO_STEER_Kp * angleError.getDegrees();
    // Limit output
    output = Util.limit(output, 0.5);
    // Apply to drive signal
    DriveSignal newSignal = new DriveSignal(signal.getLeft() - output, signal.getRight() + output);
    mrearLeftCIM.set(ControlMode.PercentOutput, newSignal.getLeft());
    mrearRightCIM.set(ControlMode.PercentOutput, newSignal.getRight());
    setBrakeMode(signal.getBrakeMode());
  }

  /**
   * Controls the motors with a closed loop velocity signal
   * @param signal The velocities to set the motors to in ticks/100ms
   * @param feedforward The feedforward gain to apply to the motors
   */
  public void setVelocity(DriveSignal signal, DriveSignal feedforward) {
    switchDriveControlModes(DriveControlMode.PATH_FOLLOWING);
    mrearLeftCIM.set(ControlMode.Velocity, signal.getLeft(), DemandType.ArbitraryFeedForward, feedforward.getLeft());
    mrearRightCIM.set(ControlMode.Velocity, signal.getRight(), DemandType.ArbitraryFeedForward, feedforward.getRight());
  }

  /**
   * Sets a trajectory for the drive to follow while in path following mode
   * @param trajectory The trajectory that the robot should follow
   */
  public void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    setTrajectory(trajectory, false);
  }

  /**
   * Sets a trajectory for the drive to follow while in path following mode
   * @param trajectory The trajectory that the robot should follow
   * @param followBackwards Whether the robot should drive backwards or not
   */
  public void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory, boolean followBackwards) {
    followPathBackwards = followBackwards;
    mMotionPlanner.reset();
    mMotionPlanner.setTrajectory(trajectory);
    switchDriveControlModes(DriveControlMode.PATH_FOLLOWING);
  }

  /**
   * Returns whether or not the drive is finished with a previously loaded trajectory
   * @return True if drive is done with the trajectory, else false
   */
  public boolean isDoneWithTrajectory() {
    if (currentControlMode != DriveControlMode.PATH_FOLLOWING) {
      return false;
    }
    return mMotionPlanner.isDone();
}

  /**
   * Sets drive to brake or coast
   * @param brake if true, drive will be set to brake. Otherwise, it will be set to coast.
   */
  public void setBrakeMode(boolean brake) {
    if (brakeMode != brake) {
      brakeMode = brake;
      NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
      mrearLeftCIM.setNeutralMode(mode);
      mfrontLeftCIM.setNeutralMode(mode);
      mrearRightCIM.setNeutralMode(mode);
      mfrontRightCIM.setNeutralMode(mode);
    }
  }

  /**
	 * Shift gearbox into high
	 */
	public void shiftHigh() {
    if (getCurrentGear() != DriveGearState.HIGH) {
      mshiftSolenoid.set(Value.kReverse);
      currentGear = DriveGearState.HIGH;
    }
	}
	
	/**
	 * Shift gearbox into low
	 */
	public void shiftLow() {
    if (getCurrentGear() != DriveGearState.LOW) {
      mshiftSolenoid.set(Value.kForward);
      currentGear = DriveGearState.LOW;
    }
	}
	
	/**
	 * Finds which gear the drive is currently in.
	 * @return the current gear state
	 */
	public DriveGearState getCurrentGear() {
		return currentGear;
  }

  /**
   * Gets the right sensor position
   * @return right sensor position in encoder ticks
   */
  public double getRightSensorPosition() {
    return mrearRightCIM.getSelectedSensorPosition(0);
  }

  /**
   * Gets the right linear position
   * @return right linear position in feet
   */
  public double getRightLinearPosition() {
    return Conversions.encoderTicksToFeet(getRightSensorPosition(), Constants.DRIVE_TICKS_PER_REV, Constants.WHEEL_CIRCUMFERENCE_INCHES);
  }

  /**
   * Gets the left sensor position
   * @return left sensor position in encoder ticks
   */
  public double getLeftSensorPosition() {
    return mrearLeftCIM.getSelectedSensorPosition(0);
  }

  /**
   * Gets the left linear position
   * @return left linear position in feet
   */
  public double getLeftLinearPosition() {
    return Conversions.encoderTicksToFeet(getLeftSensorPosition(), Constants.DRIVE_TICKS_PER_REV, Constants.WHEEL_CIRCUMFERENCE_INCHES);
  }

  /**
   * Gets the right side sensor velocity
   * @return right sensor velocity in native encoder units (pulses/100ms)
   */
  public double getRightSensorVelocity() {
    return mrearRightCIM.getSelectedSensorVelocity(0);
  }

  /**
   * Gets the right side linear velocity
   * @return right linear velocity in ft/s
   */
  public double getRightLinearVelocity() {
    return Conversions.encoderVelocityToLinearVelocity(getRightSensorVelocity() * 10.0, Constants.DRIVE_TICKS_PER_REV, Constants.WHEEL_CIRCUMFERENCE_INCHES);
  }

  /**
   * Gets the left side sensor velocity
   * @return left sensor velocity in native encoder units (pulses/100ms)
   */
  public double getLeftSensorVelocity() {
    return mrearLeftCIM.getSelectedSensorVelocity(0);
  }

  /**
   * Gets the left side linear velocity
   * @return left linear velocity in ft/s
   */
  public double getLeftLinearVelocity() {
    return Conversions.encoderVelocityToLinearVelocity(getLeftSensorVelocity() * 10.0, Constants.DRIVE_TICKS_PER_REV, Constants.WHEEL_CIRCUMFERENCE_INCHES);
  }

  /**
   * Gets the linear velocity of the robot (forward and backward motion)
   * @return linear velocity in ft/s
   */
  public double getLinearVelocity() {
    // The linear velocity of the robot is the average of the two drive sides' linear velocities.
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2;
  }

  /**
   * Gets the angular velocity of the robot (rate of turn) based on encoder feedback
   * @return angular velocity in radians/s (positive = CCW)
   */
  public double getAngularVelocity() {
    return (getRightLinearVelocity() - getLeftLinearVelocity()) / (Constants.WHEEL_BASE_INCHES / 12);
  }

  /**
   * Gets the heading of the robot in degrees
   * @return heading of the robot in degrees (positive = CCW)
   */
  public double getHeadingDegrees() {
    return (-mgyro.getAngle() + gyroOffset) % 360;
  }

  /**
   * Gets the heading of the robot
   * @return heading of the robot as a Rotation2d object
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Sets the heading of the robot
   * @param heading The new heading for the robot
   */
  public void setHeading(Rotation2d heading) {
    // Reset the gyro accumulator to 0
    mgyro.reset();
    // Set the gyro offset
    gyroOffset = heading.getDegrees();
  }

  /**
   * Resets the heading of the robot so that it is currently facing 0 degrees.
   */
  public void resetHeading() {
    setHeading(Rotation2d.fromDegrees(0.0));
  }

  /**
   * Resets the encoder positions
   */
  public void resetEncoders() {
    mrearLeftCIM.setSelectedSensorPosition(0, 0, 0);
    mrearRightCIM.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Resets all sensors (encoder positions and gyro heading)
   */
  public void zeroSensors() {
    resetHeading();
    resetEncoders();
  }

  /**
   * Updates the path follower. This method should be called periodically while the robot
   * is following a path.
   */
  public void  updatePathFollower() {
    if (currentControlMode == DriveControlMode.PATH_FOLLOWING) {
      final double now = Timer.getFPGATimestamp();

      DriveMotionPlanner.Output output = mMotionPlanner.update(now, Robot.stateEstimator.getRobotState().getFieldToVehicle(now));

      double left_command = Conversions.angularVelocityToEncoderVelocity(output.left_velocity, Constants.DRIVE_TICKS_PER_REV) / 10.0;
      double right_command = Conversions.angularVelocityToEncoderVelocity(output.right_velocity, Constants.DRIVE_TICKS_PER_REV) / 10.0;
      double left_accel = Conversions.angularVelocityToEncoderVelocity(output.left_accel, Constants.DRIVE_TICKS_PER_REV) / 10.0 / 1000.0;
      double right_accel = Conversions.angularVelocityToEncoderVelocity(output.right_accel, Constants.DRIVE_TICKS_PER_REV) / 10.0 / 1000.0;
      double left_d = Constants.LOW_GEAR_VELOCITY_Kd * left_accel / 1023.0;
      double right_d = Constants.LOW_GEAR_VELOCITY_Kd * right_accel / 1023.0;
      setVelocity(new DriveSignal(left_command, right_command), new DriveSignal(output.left_feedforward_voltage / 12.0 + left_d, output.right_feedforward_voltage / 12.0 + right_d));
    }
    else {
      DriverStation.reportError("Drive is not in path following state", false);
    }
  }

  /**
   * Reloads TalonSRX PID gains for closed loop velocity control.
   */
  public void reloadGains() {
    mrearLeftCIM.config_kP(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Kp, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_kI(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Ki, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_kD(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Kd, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_kF(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Kf, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_IntegralZone(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_IZONE, Constants.LONG_CAN_TIMEOUT_MS);

    mrearRightCIM.config_kP(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Kp, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_kI(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Ki, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_kD(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Kd, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_kF(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_Kf, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_IntegralZone(kLowGearVelocityControlSlot, Constants.LOW_GEAR_VELOCITY_IZONE, Constants.LONG_CAN_TIMEOUT_MS);

    mrearLeftCIM.config_kP(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Kp, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_kI(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Ki, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_kD(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Kd, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_kF(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Kf, Constants.LONG_CAN_TIMEOUT_MS);
    mrearLeftCIM.config_IntegralZone(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_IZONE, Constants.LONG_CAN_TIMEOUT_MS);

    mrearRightCIM.config_kP(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Kp, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_kI(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Ki, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_kD(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Kd, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_kF(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_Kf, Constants.LONG_CAN_TIMEOUT_MS);
    mrearRightCIM.config_IntegralZone(kHighGearVelocityControlSlot, Constants.HIGH_GEAR_VELOCITY_IZONE, Constants.LONG_CAN_TIMEOUT_MS);
  }

  /**
   * Handles auto shifting.
   */
  private void handleAutoShifting() {
    final double linear_velocity = Math.abs(getLinearVelocity());
    final double angular_velocity = Math.abs(getAngularVelocity());

    if (currentGear == DriveGearState.HIGH && linear_velocity < Constants.AUTO_SHIFT_DOWN_SP_THRESH && angular_velocity < Constants.AUTO_SHIFT_DOWN_ANGULAR_THRESH) {
      shiftLow();
    }
    else if (linear_velocity > Constants.AUTO_SHIFT_UP_SP_THRESH) {
      shiftHigh();
    }
  }

  /**
   * This method switches the current mode of operation to the desired mode.
   * @param newMode the DriveControlMode to switch to.
   */
  private void switchDriveControlModes(DriveControlMode newMode) {
    switch(newMode) {
      case OPEN_LOOP:
        if (currentControlMode != DriveControlMode.OPEN_LOOP) {
          // Disable brake mode
          setBrakeMode(true);
          // Enable safety timeout
          mrearLeftCIM.setSafetyEnabled(true);
          mrearRightCIM.setSafetyEnabled(true);
          // Set neutral deadband
          mrearLeftCIM.configNeutralDeadband(kNeutralDeadband, 0);
          mrearRightCIM.configNeutralDeadband(kNeutralDeadband, 0);
          currentControlMode = DriveControlMode.OPEN_LOOP;
        }
        break;
      case PATH_FOLLOWING:
        if (currentControlMode != DriveControlMode.PATH_FOLLOWING) {
          // Enable brake mode
          setBrakeMode(true);
          // Disable safety timeout
          mrearLeftCIM.setSafetyEnabled(false);
          mrearRightCIM.setSafetyEnabled(false);
          // Shift into low gear
          shiftLow();
          // Select gain profile
          mrearLeftCIM.selectProfileSlot(kHighGearVelocityControlSlot, 0);
          mrearRightCIM.selectProfileSlot(kHighGearVelocityControlSlot, 0);
          // Zero neutral deadband
          mrearLeftCIM.configNeutralDeadband(0.0, 0);
          mrearRightCIM.configNeutralDeadband(0.0, 0);
          currentControlMode = DriveControlMode.PATH_FOLLOWING;
        }
        break;
        case AUTO_STEER:
        if (currentControlMode != DriveControlMode.AUTO_STEER) {
          // Enable brake mode
          setBrakeMode(true);
          // Enable safety timeout
          mrearLeftCIM.setSafetyEnabled(true);
          mrearRightCIM.setSafetyEnabled(true);
          // Set neutral deadband
          mrearLeftCIM.configNeutralDeadband(kNeutralDeadband, 0);
          mrearRightCIM.configNeutralDeadband(kNeutralDeadband, 0);

          // Reset the last yaw angle to null (no target found)
          m_lastAngleSetpoint = null;
          currentControlMode = DriveControlMode.AUTO_STEER;
        }
    }
  }

}
