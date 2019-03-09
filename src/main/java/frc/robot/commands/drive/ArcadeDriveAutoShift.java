package frc.robot.commands.drive;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Arcade drive with auto shifting using two joysticks. This command is designed
 * to be run periodically.
 */
public class ArcadeDriveAutoShift extends Command {
  public ArcadeDriveAutoShift() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double throttle = -Robot.m_oi.getThrottleJoystick().getY();
    double steering = Robot.m_oi.getSteeringJoystick().getX();
    Robot.drive.arcadeDrive(throttle, steering, true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
