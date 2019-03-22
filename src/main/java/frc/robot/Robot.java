/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team254.geometry.Rotation2d;
import frc.util.CurvatureDriveHelper;
import frc.robot.commands.auto.FrontRocketToFeeder;
import frc.robot.commands.drive.DriveAutoSteer;
import frc.robot.commands.drive.OpenLoopDrive;
import frc.robot.paths.PathCommandSelector;
import frc.robot.paths.TrajectoryGenerator;
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

  // States for the sandstorm state machine
  public enum AutoState { PATH_1, PATH_1_RETURN, PATH_2, PATH_2_RETURN, MANUAL_CONTROL }
  private static AutoState m_currentAutoState = AutoState.PATH_1;
  private static AutoState m_nextState = AutoState.PATH_1_RETURN;
  private static PathCommandSelector m_pathSelector;

  // Choosers for selecting robot starting position and paths to run
  SendableChooser<Integer> m_startPositionChooser = new SendableChooser<>();
  SendableChooser<Integer> m_path1Chooser = new SendableChooser<>();
  SendableChooser<Integer> m_path2Chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Reset the state estimator
    stateEstimator.reset(Timer.getFPGATimestamp(), RobotState.generateFieldToVehicle(0, 0, 0));

    // Set up dashboard choosers
    m_startPositionChooser.setDefaultOption("Right lvl 1", 0);
    m_startPositionChooser.addOption("Left lvl 1", 1);
    m_startPositionChooser.addOption("Middle lvl 1", 2);
    m_startPositionChooser.addOption("Right lvl 2", 3);
    m_startPositionChooser.addOption("Left lvl 2", 4);
    SmartDashboard.putData("Starting Position", m_startPositionChooser);

    m_path1Chooser.setDefaultOption("Front rocket", 0);
    m_path1Chooser.addOption("Back rocket", 1);
    m_path1Chooser.addOption("Front cargoship", 2);
    m_path1Chooser.addOption("Side cargoship", 3);
    SmartDashboard.putData("Path 1", m_path1Chooser);

    m_path2Chooser.setDefaultOption("Front rocket", 0);
    m_path2Chooser.addOption("Back rocket", 1);
    m_path2Chooser.addOption("Front cargoship", 2);
    m_path2Chooser.addOption("Side cargoship", 3);
    SmartDashboard.putData("Path 2", m_path2Chooser);

    // Instantiate camera server for usb camera on RoboRio
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera.setResolution(320, 240);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("carriage-cam", 160, 120);

      Mat source = new Mat();
      Mat output = new Mat();

      long test;
      while (!Thread.interrupted()) {
        test = cvSink.grabFrame(source);
        // Reduce size
        if (test != 0) {
          Imgproc.resize(source, output, new Size(160, 120));
          // Convert to grayscale
          Imgproc.cvtColor(output, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
        }
      }

    }).start();

    // Generate paths
    TrajectoryGenerator.getInstance().generateTrajectories();
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
    // Get choosable parameters from the dashboard
    int startPosition = m_startPositionChooser.getSelected();
    int path1 = m_path1Chooser.getSelected();
    int path2 = m_path2Chooser.getSelected();

    // Construct path selector object
    m_pathSelector = new PathCommandSelector(startPosition, path1, path2);

    // Reset the sandstorm state machine
    m_currentAutoState = AutoState.PATH_1;

    // Start the first path
    m_pathSelector.getFirstPathCommand().start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Sandstorm state machine
    switch(m_currentAutoState) {
      case MANUAL_CONTROL:
        teleopPeriodic();
        // If driver presses resume button, move to next state
        if (m_oi.getSteeringJoystick().getRawButtonPressed(2)) {
          m_currentAutoState = m_nextState;
          if (m_nextState == AutoState.PATH_1) {
            m_pathSelector.getFirstPathCommand().start();
          }
          else if (m_nextState == AutoState.PATH_1_RETURN) {
            m_pathSelector.getSecondPathCommand().start();
          }
          else if (m_nextState == AutoState.PATH_2) {
            m_pathSelector.getThirdPathCommand().start();
          }
          else {
            m_pathSelector.getFourthPathCommand().start();
          }
        }
        break;
      
      case PATH_1:
        Scheduler.getInstance().run();
        // Return to manual control when driver presses quick turn button
        if (m_oi.getSteeringJoystick().getRawButtonPressed(1)) {
          m_currentAutoState = AutoState.MANUAL_CONTROL;
          m_nextState = AutoState.PATH_1_RETURN;
        }
        break;

      case PATH_1_RETURN:
        Scheduler.getInstance().run();
        // Return to manual control when driver presses quick turn button
        if (m_oi.getSteeringJoystick().getRawButtonPressed(1)) {
          m_currentAutoState = AutoState.MANUAL_CONTROL;
          m_nextState = AutoState.PATH_2;
        }
        break;

      case PATH_2:
        Scheduler.getInstance().run();
        // Return to manual control when driver presses quick turn button
        if (m_oi.getSteeringJoystick().getRawButtonPressed(1)) {
          m_currentAutoState = AutoState.MANUAL_CONTROL;
          m_nextState = AutoState.PATH_2_RETURN;
        }
        break;

      case PATH_2_RETURN:
        Scheduler.getInstance().run();
        // Return to manual control when driver presses quick turn button
        if (m_oi.getSteeringJoystick().getRawButtonPressed(1)) {
          m_currentAutoState = AutoState.MANUAL_CONTROL;
          m_nextState = AutoState.PATH_2_RETURN;
        }
        break;
    }
  }

  @Override
  public void teleopInit() {
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
