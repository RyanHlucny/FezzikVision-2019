/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robot;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;

/**
 * Sets the current game piece pursuit to CARGO_POD.
 */
public class SetGamePiecePursuitBall extends InstantCommand {

  public SetGamePiecePursuitBall() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.setGamePiecePursuit(GamePieceMode.CARGO);
  }

}
