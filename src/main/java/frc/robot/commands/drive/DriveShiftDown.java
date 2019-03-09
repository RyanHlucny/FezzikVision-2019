package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * This command shifts the drivetrain gearboxes into low gear. If they are already in low gear,
 * this command does nothing.
 */
public class DriveShiftDown extends InstantCommand {

  public DriveShiftDown() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.drive.shiftLow();
  }

}
