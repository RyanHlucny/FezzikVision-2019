package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * This command shifts the drivetrain gearboxes into low gear. If they are already in
 * high gear, this command does nothing.
 */
public class DriveShiftUp extends InstantCommand {

  public DriveShiftUp() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.drive.shiftHigh();
  }

}
