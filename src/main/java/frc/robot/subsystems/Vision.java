/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotState;

/**
 * Vision subsystem; controls the two raspberry pi 3 B+ coprocessors and handles vision processing
 * on the robot.
 */
public class Vision extends Subsystem {
  // Constants
  // NetworkTable table names: these must match the names in the Raspberry Pi vision code!
  private static final String k_cargoTableName = "CargoCam";
  private static final String k_hatchTableName = "HatchCam";
  // Name of entry to listen to for target updates
  private static final String k_targetKey = "target";
  // Name of entry to set camera offsets on
  private static final String k_offsetKey = "offsets";
  // Size of the target info array returned by the raspberry pis.
  private static final int k_targetInfoArraySize = 9;

  // Camera offsets from center of the robot (in inches)
  private static final Translation2d k_cargoCamOffset = new Translation2d(2.75, 8.25);
  private static final Translation2d k_hatchCamOffset = new Translation2d(-6.0, 12.25); // y was 12.25

  // Vision States
  public enum VisionState { DRIVE_MODE, PROCESSING_MODE }

  // Structure for holding vision target information
  public class VisionTarget {
    public VisionTarget() {}
    /* Copy Constructor */
    public VisionTarget(VisionTarget target) {
      this.centerPixel = target.centerPixel;
      this.yaw = target.yaw;
      this.pitch = target.pitch;
      this.xOffset = target.xOffset;
      this.yOffset = target.yOffset;
      this.skewAngle = target.skewAngle;
      this.width = target.width;
      this.height = target.height;
      this.latency = target.latency;
      this.timestamp = target.timestamp;
    }
    public double centerPixel; // Horizontal pixel coordinate of the center of the target (camera dependent)
    public double yaw; // yaw angle from the camera to the center of the target
    public double pitch; // pitch angle from the camera to the center of the target
    public double xOffset; // horizontal offset of target from the camera in the x direction (ahead and behind camera) in inches
    public double yOffset; // horizontal offset of target from the camera in the y direction (left and right of camera) in inches
    public double skewAngle; // rotation of the target around the z axis (up and down)
    public double width; // width of the target in inches
    public double height; // height of the target in inches

    public double latency; // latency of information due to image acquisition and processing (ms)
    public double timestamp; // time when this information was received by the roboRIO (FPGATimestamp) in seconds
  }

  // Components
  private final NetworkTableInstance m_nTableInst = NetworkTableInstance.getDefault();
  private final NetworkTable m_hatchTable = m_nTableInst.getTable(k_hatchTableName);
  private final NetworkTable m_cargoTable = m_nTableInst.getTable(k_cargoTableName);

  // State variables
  private VisionState m_currentState;
  private VisionTarget m_currentHatchTarget = null;
  private VisionTarget m_currentCargoTarget = null;

  /* Default constructor */
  public Vision() {
    // Initialize mode to drive mode
    m_currentState = VisionState.PROCESSING_MODE;
    switchStates(VisionState.DRIVE_MODE);

    // Add listeners for vision targets reported by the raspberry pis
    m_hatchTable.addEntryListener(k_targetKey, (table, key, entry, value, flags) -> {
      VisionTarget tempTarget = camListener(value);
      // If target found and we had one before, perform position filtering
      if (tempTarget != null && m_currentHatchTarget != null) {
        // Only use new target if yaw angle hasn't changed too much
        if (Math.abs(tempTarget.yaw - m_currentHatchTarget.yaw) < 6) {
          m_currentHatchTarget = tempTarget;
        }
      }
      else {
        m_currentHatchTarget = tempTarget;
      }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    m_cargoTable.addEntryListener(k_targetKey, (table, key, entry, value, flags) -> {
      VisionTarget tempTarget = camListener(value);
      // If target found and we had one before, perform position filtering
      if (tempTarget != null && m_currentCargoTarget != null) {
        // Only use new target if yaw angle hasn't changed too much
        if (Math.abs(tempTarget.yaw - m_currentCargoTarget.yaw) < 6) {
          m_currentCargoTarget = tempTarget;
        }
      }
      else {
        m_currentCargoTarget = tempTarget;
      }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Add camera offsets to the network tables
    if (!m_hatchTable.getEntry(k_offsetKey).setDoubleArray(new double[]{k_hatchCamOffset.x(), k_hatchCamOffset.y()})) {
      DriverStation.reportError("Unable to set hatch camera offsets.", false);
    }
    if (!m_cargoTable.getEntry(k_offsetKey).setDoubleArray(new double[]{k_cargoCamOffset.x(), k_cargoCamOffset.y()})) {
      DriverStation.reportError("Unable to set cargo camera offsets.", false);
    }
  }

  @Override
  public void initDefaultCommand() {
  }

  /**
   * Finds the distance from the center of the robot to the nearest cargo target
   * @return Distance to the nearest cargo target (inches), or null if no target in sight.
   */
  public Double getDistanceToCargoTarget() {
    Pose2d targetPose = getCargoTargetPose();
    if (targetPose == null) {
      return null;
    }

    Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    return (Double) robotPose.distance(targetPose);
  }

  /**
   * Finds the distance from the center of the robot to the nearest hatch target
   * @return Distance to the nearest hatch target (inches), or null if no target in sight.
   */
  public Double getDistanceToHatchTarget() {
    Pose2d targetPose = getHatchTargetPose();
    if (targetPose == null) {
      return null;
    }

    Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    return (Double) robotPose.distance(targetPose);
  }

  /**
   * Finds the distance from the bumper on the side of the cargo mechanism to the
   * nearest cargo target
   * @return Distance from bumper to the nearest cargo target (inches), or null if no
   * target is in sight.
   */
  public Double getFrontBumperDistanceToCargoTarget() {
    Pose2d targetPose = getCargoTargetPose();
    if (targetPose == null) {
      return null;
    }

    Pose2d frontBumperPose = Pose2d.fromTranslation(new Translation2d(Constants.ROBOT_FRAME_LENGTH_PLUS_BUMPERS/2, 0));
    return (Double) frontBumperPose.distance(targetPose);
  }

  /**
   * Finds the distance from the bumper on the side of the hatch mechanism to the
   * nearest hatch target
   * @return Distance from bumper to the nearest hatch target (inches), or null if no
   * target is in sight.
   */
  public Double getFrontBumperDistanceToHatchTarget() {
    Pose2d targetPose = getHatchTargetPose();
    if (targetPose == null) {
      return null;
    }

    Pose2d frontBumperPose = Pose2d.fromTranslation(new Translation2d(Constants.ROBOT_FRAME_LENGTH_PLUS_BUMPERS/2, 0));
    return (Double) frontBumperPose.distance(targetPose);
  }

  /**
   * Finds the 2d pose of the target currently visible from the cargo camera with respect to the camera
   * @return Pose2d of the currently visible target, or null if no target is visible.
   */
  public Pose2d getCargoTargetPoseWRTCamera() {
    if (m_currentCargoTarget == null) {
      return null;
    }
    
    return convertTargetToPose(m_currentCargoTarget);
  }

  /**
   * Finds the 2d pose of the target currently visible from the hatch camera with respect to the camera
   * @return Pose2d of the currently visible target, or null if no target is visible.
   */
  public Pose2d getHatchTargetPoseWRTCamera() {
    if (m_currentHatchTarget == null) {
      return null;
    }

    return convertTargetToPose(m_currentHatchTarget);
  }

  /**
   * Finds the yaw angle of the center of the target relative to the center of the camera
   * @return Rotation2d of the currently visible target's yaw angle, or null if no target is visible.
   */
  public Rotation2d getCargoYawAngleWRTCamera() {
    if (m_currentCargoTarget == null) {
      return null;
    }

    return Rotation2d.fromDegrees(m_currentCargoTarget.yaw);
  }

  /**
   * Finds the yaw angle of the center of the target relative to the center of the camera
   * @return Rotation2d of the currently visible target's yaw angle, or null if no target is visible.
   */
  public Rotation2d getHatchYawAngleWRTCamera() {
    if (m_currentHatchTarget == null) {
      return null;
    }
    
    return Rotation2d.fromDegrees(m_currentHatchTarget.yaw);
  }

  /**
   * Finds the 2d pose of the target currently visible from the cargo camera relative to the robot's
   * center
   * @return Pose2d of the currently visible target, or null if no target is visible.
   */
  public Pose2d getCargoTargetPose() {
    Pose2d targetWRTCamera = getCargoTargetPoseWRTCamera();
    if (targetWRTCamera == null) {
      return null;
    }

    return targetWRTCamera.transformBy(Pose2d.fromTranslation(k_cargoCamOffset));
  }

  /**
   * Finds the 2d pose of the target currently visible from the hatch camera relative to the robot's
   * center
   * @return Pose2d of the currently visible target, or null if no target is visible.
   */
  public Pose2d getHatchTargetPose() {
    Pose2d targetWRTCamera = getHatchTargetPoseWRTCamera();
    if (targetWRTCamera == null) {
      return null;
    }

    return targetWRTCamera.transformBy(Pose2d.fromTranslation(k_hatchCamOffset));
  }

  /**
   * Finds the yaw angle from the center of the robot to the nearest cargo target currently
   * in sight
   * @return Rotation2d representing the yaw angle to the nearest cargo target, or null if no target is visible.
   */
  public Rotation2d getCargoYawAngle() {
    Pose2d targetPose = getCargoTargetPose();
    if (targetPose == null) {
      return null;
    }

    return new Rotation2d(targetPose.getTranslation(), true);
  }

  /**
   * Finds the latency-compensated yaw angle from the center of the robot to the cargo target
   * @param state robot state
   * @return The latency-compensated yaw angle, or null if no target is visible
   */
  public Rotation2d getCompensatedCargoYaw(RobotState state) {
    Rotation2d currentYaw = getCargoYawAngle();
    VisionTarget target = m_currentCargoTarget;
    if (target == null || currentYaw == null) {
      return null;
    }

    // Calculate the change in robot's angle since the image was taken
    Rotation2d change = getLatencyYawAdjustment(target, state);
    // Subtract the change from the current yaw angle to get a better estimate
    return change.inverse().rotateBy(currentYaw);
  }

  /**
   * Finds the yaw angle from the center of the robot to the nearest hatch target currently
   * in sight
   * @return Rotation2d representing the yaw angle to the nearest hatch target, or null if no target is visible.
   */
  public Rotation2d getHatchYawAngle() {
    Pose2d targetPose = getHatchTargetPose();
    if (targetPose == null) {
      return null;
    }

    return new Rotation2d(targetPose.getTranslation(), true);
  }

  /**
   * Finds the latency-compensated yaw angle from the center of the robot to the hatch target
   * @param state robot state
   * @return The latency-compensated yaw angle, or null if no target is visible
   */
  public Rotation2d getCompensatedHatchYaw(RobotState state) {
    Rotation2d currentYaw = getHatchYawAngle();
    VisionTarget target = m_currentHatchTarget;
    if (target == null || currentYaw == null) {
      return null;
    }

    // Calculate the change in robot's angle since the image was taken
    Rotation2d change = getLatencyYawAdjustment(target, state);
    // Subtract the change from the current yaw angle to get a better estimate
    return change.inverse().rotateBy(currentYaw);
  }

  /**
   * Gets the current vision state
   * @return VisionState of the vision subsystem
   */
  public VisionState getVisionMode() {
    return m_currentState;
  }

  /**
   * Sets Vision subsystem to the desired mode
   * @param mode the mode to set the vision subsystem to
   */
  public void setVisionMode(VisionState mode) {
    switchStates(mode);
  }

  /**
   * Method that responds to a change in target information reported by a raspberry pi.
   * It uses the target information to create a new VisionTarget object.
   * Note that this object represents the target relative to the camera. If the target
   * pose relative to the robot is required, additional processing will need to be done.
   * @param value The value that has changed
   * @return VisionTarget object representing the vision target currently in sight or null if no
   * targets are in sight.
   */
  private VisionTarget camListener(NetworkTableValue value) {
    VisionTarget target;
    // Get array of target information from the raspberry pi
    double[] targetArray;
    try {
      targetArray = value.getDoubleArray();
    }
    catch (ClassCastException e) {
      targetArray = new double[1];
      targetArray[0] = 0.0;
      DriverStation.reportError("Target info received was of the incorrect data type!", false);
    }

    // Is there a target in sight?
    if (targetArray.length != k_targetInfoArraySize) {
      // If not, current target should be null
      target = null;
    }
    else {
      // If so, create a new target object
      target = new VisionTarget();
      target.centerPixel = targetArray[0];
      target.yaw = targetArray[1];
      target.pitch = targetArray[2];
      target.xOffset = targetArray[3];
      target.yOffset = targetArray[4];
      target.skewAngle = targetArray[5];
      target.width = targetArray[6];
      target.height = targetArray[7];
      target.latency = targetArray[8];
      target.timestamp = Timer.getFPGATimestamp();
    }

    return target;
  }

  /**
   * Helper method that converts a VisionTarget object to a Pose2d.
   * @param target the VisionTarget to convert
   * @return Pose2d of the target
   */
  private Pose2d convertTargetToPose(VisionTarget target) {
    return new Pose2d(new Translation2d(target.xOffset, target.yOffset), Rotation2d.fromDegrees(target.skewAngle));
  }

  /*
  /**
   * Gets the field-relative location of the target, adjusted for camera latency
   * @param target the vision target to compensate
   * @param state the RobotState of the robot
   * @return field-relative location of the target
   *
  private Translation2d getLatencyCompAdjustment(boolean isCargoTarget, RobotState state) {
    // TODO: This method needs to be checked over and throughly tested. We aren't sure if it's working and are not using it.
    final double now = Timer.getFPGATimestamp();
    final VisionTarget target = (isCargoTarget) ? m_currentCargoTarget : m_currentHatchTarget;
    // total latency is the processing latency + time since info was received
    final double totalLatency = target.latency / 1000 + (now - target.timestamp);
    // Get the robot's approximate pose at time of image capture
    final Pose2d capturePose = state.getFieldToVehicle(now - totalLatency);
    // Get the robot's current pose
    final Pose2d currentPose = state.getFieldToVehicle(now);
    // Find the difference in robot's pose since time of image capture
    // This is the equivalent of subtracting the capture pose from the current pose
    final Pose2d poseChange = capturePose.inverse().transformBy(currentPose);

    // Get the target's position relative to robot at the time of capture
    final Translation2d relTargetCaptureLoc = (isCargoTarget) ? getCargoTargetPose().getTranslation(): getHatchTargetPose().getTranslation().inverse();
    // Convert the robot-relative target location to a field-relative location
    final Translation2d targetCaptureLoc = relTargetCaptureLoc.rotateBy(capturePose.getRotation());
    // Estimate the target's actual position based on the robot's pose change
    // This is the equivalent of subtracting the position change from the target's capture position
    final Translation2d targetActualLoc = poseChange.getTranslation().inverse().translateBy(targetCaptureLoc);
    // TODO: Make this method calculate skew angle as well

    return targetActualLoc;
  }
  */

  /**
   * Gets the difference in the robot's yaw angle between the time when the image was captured
   * and now. Subtract this value from the read yaw angle to get a better estimate.
   * @param isCargoTarget whether or not to use the cargo vision target
   * @param state current state of the robot
   * @return the change in the robot's yaw angle
   */
  private Rotation2d getLatencyYawAdjustment(final VisionTarget target, final RobotState state) {
    final double now = Timer.getFPGATimestamp();
    // total latency is the processing latency + time since info was received
    final double totalLatency = target.latency / 1000 + (now - target.timestamp);
    // Get the robot's approximate angle at time of image capture
    final Rotation2d captureAngle = state.getFieldToVehicle(now - totalLatency).getRotation();
    // Get the robot's current angle
    final Rotation2d currentAngle = state.getFieldToVehicle(now).getRotation();
    // Find the difference in the robot's angle since the time of image capture
    return captureAngle.inverse().rotateBy(currentAngle);
  }

  private void switchStates(VisionState desiredState) {
    switch(desiredState) {
      case DRIVE_MODE:
        if (m_currentState != VisionState.DRIVE_MODE) {
          m_currentState = VisionState.DRIVE_MODE;
          
          m_currentHatchTarget = null;
          m_currentCargoTarget = null;
        }
        break;

      case PROCESSING_MODE:
        if (m_currentState != VisionState.PROCESSING_MODE) {
          m_currentState = VisionState.PROCESSING_MODE;
        }
        break;
    }
  }

}
