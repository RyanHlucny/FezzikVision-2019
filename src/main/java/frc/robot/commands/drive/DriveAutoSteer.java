/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.lib.team254.geometry.Rotation2d;
import frc.team5172.lib.util.DriveSignal;
import frc.robot.Robot;

/**
 * Drive command for the Drive subsystem that uses auto steering. 
 * This command is designed to be called periodically.
 */
public class DriveAutoSteer extends InstantCommand {
  private DriveSignal signal;
  private Rotation2d headingSetpoint;

  /**
   * Add your docs here.
   */
  public DriveAutoSteer(DriveSignal signal, Rotation2d headingSetpoint) {
    super();
    requires(Robot.drive);
    this.signal = signal;
    this.headingSetpoint = headingSetpoint;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.drive.driveAutoSteer(signal, headingSetpoint);
  }

}
