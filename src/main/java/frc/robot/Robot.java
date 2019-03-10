/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Files;
import java.nio.file.Paths;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team254.geometry.Rotation2d;
import frc.util.CurvatureDriveHelper;
import frc.robot.commands.drive.DriveAutoSteer;
import frc.robot.commands.drive.OpenLoopDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionState;
import frc.team5172.lib.util.DriveSignal;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drive drive = new Drive();
  public static Vision vision = new Vision();

  // This must be instantiated after the drive subsystem
  public static RobotStateEstimator stateEstimator = new RobotStateEstimator();

  public static OI m_oi = new OI();

  // Helper objects
  private CurvatureDriveHelper curvatureDrive = new CurvatureDriveHelper();

  // Robot-wide Game Piece Mode
  public enum GamePieceMode { CARGO, HATCH }
  public static GamePieceMode m_currentGamePieceMode = GamePieceMode.HATCH;

  Command m_autonomousCommand;
  // SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Reset the state estimator
    stateEstimator.reset(Timer.getFPGATimestamp(), RobotState.generateFieldToVehicle(0, 0, 0));

    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    // SmartDashboard.putData("Auto mode", m_chooser);

    // Instantiate camera server for usb camera on RoboRio
    /*
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    try{
      String cameraConfig = new String(Files.readAllBytes(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "CameraConfig.json")), "UTF-8");
      camera.setConfigJson(cameraConfig);
    }
    catch (Exception e) {
      DriverStation.reportError("Unable to load camera config from Json file.", false);
    } */

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera.setResolution(320, 240);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("carriage-cam", 160, 120);

      Mat source = new Mat();
      Mat output = new Mat();

      while (!Thread.interrupted()) {
        cvSink.grabFrame(source);
        // Reduce size
        Imgproc.resize(source, output, new Size(160, 120));
        // Convert to grayscale
        Imgproc.cvtColor(output, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }

    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Shows current vision mode on the smartdashboard.
    SmartDashboard.putBoolean("Vision Processing Enabled", vision.getVisionMode() == VisionState.PROCESSING_MODE);
    SmartDashboard.putBoolean("Hatch Mode", getGamePiecePursuit() == GamePieceMode.HATCH);
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_chooser.getSelected();
    m_autonomousCommand = null;

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }

    // Reset the state estimator
    stateEstimator.reset(Timer.getFPGATimestamp(), RobotState.generateFieldToVehicle(0, 0, 0));
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Reset the state estimator
    stateEstimator.reset(Timer.getFPGATimestamp(), RobotState.generateFieldToVehicle(0, 0, 0));
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Update the state estimator
    stateEstimator.update(Timer.getFPGATimestamp());

    // Open loop curvature drive
    DriveSignal signal = curvatureDrive.curvatureDrive(m_oi.getThrottleJoystick().getRawAxis(1), m_oi.getSteeringJoystick().getRawAxis(0),
        m_oi.getSteeringJoystick().getRawButton(1), drive.getCurrentGear() == Drive.DriveGearState.HIGH);
    Command driveCommand;
    // Check to see if we're in vision steering mode
    if (vision.getVisionMode() == VisionState.PROCESSING_MODE) {
      // Find heading setpoint using vision
      Rotation2d yawAngle;
      if (getGamePiecePursuit() == GamePieceMode.CARGO) {
        yawAngle = vision.getCompensatedCargoYaw(stateEstimator.getRobotState());
      }
      else {
        yawAngle = vision.getCompensatedHatchYaw(stateEstimator.getRobotState());
      }
      Rotation2d headingSetpoint;
      if (yawAngle != null) {
        headingSetpoint = drive.getHeading().rotateBy(yawAngle);
      }
      else {
        headingSetpoint = null;
      }
      // activate auto steering
      driveCommand = new DriveAutoSteer(signal, headingSetpoint);
    }
    else {
      driveCommand = new OpenLoopDrive(signal, true);
    }

    driveCommand.start();
    driveCommand.close();

    // Run scheduled commands
    Scheduler.getInstance().run();
  }

  /**
   * This function is called each time the robot enters into test mode
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * @return Which game piece we're currently pursuing
   */
  public static GamePieceMode getGamePiecePursuit() {
    return m_currentGamePieceMode;
  }
  
  /**
   * Sets the current game piece pursuit
   * @param gamePiecePursuit game piece you want to pursue
   */
  public static void setGamePiecePursuit (GamePieceMode gamePiecePursuit) {
    m_currentGamePieceMode = gamePiecePursuit;
  }
}
