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
import frc.robot.subsystems.PlateGrabber.ClawGrabMode;

/**
 * Add your docs here.
 */
public class FeederStationPreset extends InstantCommand {
  /**
   * Add your docs here.
   */
  public FeederStationPreset() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    // code for setting the elevator preset for grabbing from feeder station
    if (Robot.getGamePiecePursuit() == GamePieceMode.HATCH) {
      Robot.plategrabber.closeClaw();
      Robot.plategrabber.switchClawGrabMode(ClawGrabMode.AUTO);
    }
  }

}
