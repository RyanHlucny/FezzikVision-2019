/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.plategrabber.AutoClawGrab;

/**
 * PlateGrabber subsystem; controls the hatch panel manipulator.
 */
public class PlateGrabber extends Subsystem {
  // PlateGrabber states
  public enum ClawState {OPEN, CLOSE}
  public enum ClawGrabMode {MANUAL, AUTO}

  // Components
  private DoubleSolenoid m_clawSolenoid;

  private final DigitalInput m_plateSwitch = new DigitalInput(1);

  // State variables
  private ClawGrabMode m_currentClawGrabMode = ClawGrabMode.MANUAL;
  private ClawState m_currentClawState;

  /** Default constructor */
  public PlateGrabber() {
    m_clawSolenoid = new DoubleSolenoid(0, RobotMap.INTAKE_OPEN_CHANNEL, RobotMap.INTAKE_CLOSE_CHANNEL); // open channel = 2, close channel = 3
    // Force PCM messages
    // Initially close claw
    m_currentClawState = ClawState.OPEN;
    closeClaw();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new AutoClawGrab());
  }

  /**
   * Opens claw to grab hatch panel
   */
  public void openClaw() {
    if (getClawState() != ClawState.OPEN) {
      m_clawSolenoid.set(Value.kForward);
      m_currentClawState = ClawState.OPEN;
    }
  }

  /**
   * Closes claw to release hatch panel
   */
  public void closeClaw() {
    if (getClawState() != ClawState.CLOSE) {
      m_clawSolenoid.set(Value.kReverse);
      m_currentClawState = ClawState.CLOSE;
    }
  }

  public void autoClawGrab() {
    if (getClawGrabMode() == ClawGrabMode.AUTO) {
      if (getPlateLimitSwitchValue() == false) {
        switchClawGrabMode(ClawGrabMode.MANUAL);
        openClaw();
      }
    }
  }

  /**
   * Finds the claw state
   * @return the current claw state
   */ 
  public ClawState getClawState() {
    return m_currentClawState;
  }

  public ClawGrabMode getClawGrabMode() {
    return m_currentClawGrabMode;
  }

  public boolean getPlateLimitSwitchValue() {
    return m_plateSwitch.get();
  }

  /**
   * Toggles claw solenoid
   */
  public void toggleClawSolenoid() {
    if (getClawState() == ClawState.CLOSE)
      openClaw();
    else if (getClawState() == ClawState.OPEN)
      closeClaw();
  }

  public void switchClawGrabMode(ClawGrabMode mode) {
    switch (mode) {
      case MANUAL:
        m_currentClawGrabMode = ClawGrabMode.MANUAL;
        break;
      
      case AUTO:
        m_currentClawGrabMode = ClawGrabMode.AUTO;
        break;
    } 
  }
}
