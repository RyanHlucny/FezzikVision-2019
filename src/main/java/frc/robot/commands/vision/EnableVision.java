/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.VisionState;

/**
 * Enables vision in the Vision subsystem.
 */
public class EnableVision extends InstantCommand {

  public EnableVision() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.vision.getVisionMode() != VisionState.PROCESSING_MODE)
      Robot.vision.setVisionMode(VisionState.PROCESSING_MODE);
  }

}
