/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.VisionState;

public class VisionHold extends Command {
  public VisionHold() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Robot.vision.getVisionMode() != VisionState.PROCESSING_MODE)
      Robot.vision.setVisionMode(VisionState.PROCESSING_MODE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (Robot.vision.getVisionMode() != VisionState.DRIVE_MODE)
      Robot.vision.setVisionMode(VisionState.DRIVE_MODE);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
