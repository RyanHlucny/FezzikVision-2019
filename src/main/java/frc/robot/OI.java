package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drive.DriveShiftDown;
import frc.robot.commands.drive.DriveShiftUp;
import frc.robot.commands.plategrabber.CloseClaw;
import frc.robot.commands.plategrabber.FeederStationPreset;
import frc.robot.commands.plategrabber.OpenClaw;
import frc.robot.commands.robot.SetGamePiecePursuitBall;
import frc.robot.commands.robot.SetGamePiecePursuitPlate;
import frc.robot.commands.vision.VisionHold;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  /* DRIVE CONTROL JOYSTICKS */
  // Throttle joystick
  private Joystick mthrottleJoy = new Joystick(0);
  private static final int autoSteerButton = 2;
  private static final int shiftDownButton = 3;
  private static final int shiftUpButton = 4;
  // Steering joystick
  private Joystick msteeringJoy = new Joystick(1);
  private static final int cargoMode = 4;
  private static final int hatchMode = 3;
  // Operator joystick
  private Joystick moperatorJoy = new Joystick(2);
  private static final int closeClaw = 2;
  private static final int openClaw = 3;
  private static final int feederPreset = 8;

  public OI() {
    // Manual shifting
    new JoystickButton(mthrottleJoy, shiftDownButton).whenPressed(new DriveShiftDown());
    new JoystickButton(mthrottleJoy, shiftUpButton).whenPressed(new DriveShiftUp());
    new JoystickButton(mthrottleJoy, autoSteerButton).whileHeld(new VisionHold());
    new JoystickButton(msteeringJoy, cargoMode).whenPressed(new SetGamePiecePursuitBall());
    new JoystickButton(msteeringJoy, hatchMode).whenPressed(new SetGamePiecePursuitPlate());
    new JoystickButton(msteeringJoy, hatchMode).whenPressed(new SetGamePiecePursuitPlate());

    new JoystickButton(moperatorJoy, closeClaw).whenPressed(new CloseClaw(true));
    new JoystickButton(moperatorJoy, openClaw).whenPressed(new OpenClaw(true));
    new JoystickButton(moperatorJoy, feederPreset).whenPressed(new FeederStationPreset());
    
    
  }


  public Joystick getThrottleJoystick() {
    return mthrottleJoy;
  }

  public Joystick getSteeringJoystick() {
    return msteeringJoy;
  }

}