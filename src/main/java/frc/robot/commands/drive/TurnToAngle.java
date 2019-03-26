/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.team254.geometry.Rotation2d;
import frc.util.CurvatureDriveHelper;
import frc.util.Debounce;
import frc.util.Util;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team5172.lib.util.DriveSignal;

public class TurnToAngle extends Command {
  private Rotation2d desiredAngle;
  private Rotation2d angleError;

  // Curvature Drive Helper Instantiation
  private CurvatureDriveHelper curvatureDrive = new CurvatureDriveHelper();
  private Debounce debounce;

  public TurnToAngle(double desiredAngleDegrees) {
    requires(Robot.drive);
    this.desiredAngle = Rotation2d.fromDegrees(desiredAngleDegrees);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    debounce = new Debounce(5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    this.angleError = Robot.drive.getHeading().rotateBy(desiredAngle);
    double output = angleError.getDegrees() * Constants.TURN_BY_ANGLE_Kp;
    output = Util.limit(output, 1);
    DriveSignal signal = curvatureDrive.curvatureDrive(0, output, true, false);
    Robot.drive.driveOpenLoop(signal, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return debounce.debounce(Math.abs(this.angleError.getDegrees()) <= 2, true);
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
