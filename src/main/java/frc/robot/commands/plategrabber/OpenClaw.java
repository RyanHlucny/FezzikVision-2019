/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.plategrabber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;

/**
 * Opens the plategrabber claw to grab a hatch panel
 */
public class OpenClaw extends InstantCommand {
  private boolean forceStateChange;

  /**
   * @param forceStateChange True to have the plategrabber claw open regardless of GamePieceMode. False means it must be in hatch mode to open.
   */
  public OpenClaw(boolean forceStateChange) {
    super();
    this.forceStateChange = forceStateChange;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (forceStateChange || Robot.getGamePiecePursuit() == GamePieceMode.HATCH) {
      Robot.plategrabber.openClaw();
    }
  }
}
