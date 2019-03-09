package frc.robot.commands.drive;

import frc.robot.Robot;
import frc.team5172.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Open loop drive command for the Drive subsystem. This command is designed to be called
 * periodically.
 */
public class OpenLoopDrive extends InstantCommand {
  private DriveSignal signal;
  private boolean autoShift;

  /**
   * Constructs an open loop drive command. This constructor automatically
   * disables auto shifting.
   * @param signal The drivesignal to send to the drive subsystem
   */
  public OpenLoopDrive(DriveSignal signal) {
    super();
    requires(Robot.drive);
    this.signal = signal;
    this.autoShift = false;
  }

  /**
   * Constructs an open loop drive command
   * @param signal The drivesignal to send to the drive subsystem
   * @param enableAutoShift Whether or not to enable auto shifting
   */
  public OpenLoopDrive(DriveSignal signal, boolean enableAutoShift) {
    super();
    requires(Robot.drive);
    this.signal = signal;
    this.autoShift = enableAutoShift;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.drive.driveOpenLoop(signal, autoShift);
  }

}
