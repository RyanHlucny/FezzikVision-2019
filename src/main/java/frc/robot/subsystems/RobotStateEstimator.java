/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.geometry.Twist2d;
import frc.robot.Kinematics;
import frc.robot.Robot;
import frc.robot.RobotState;

/**
 * This subsystem is responsible for updating the RobotState. Its update method
 * should be called periodically when state information is required.
 */
public class RobotStateEstimator extends Subsystem {
  // Private fields
  private RobotState robot_state = new RobotState();
  private Drive robot_drive = Robot.drive;
  private double left_encoder_prev_distance = 0.0;
  private double right_encoder_prev_distance = 0.0;

  @Override
  protected void initDefaultCommand() {
    // No default command
  }

  public void reset(double timestamp, Pose2d startingPose) {
    // Set previous distance values
    left_encoder_prev_distance = robot_drive.getLeftLinearPosition() * 12.0;
    right_encoder_prev_distance = robot_drive.getRightLinearPosition() * 12.0;

    // Reset robot state
    robot_state.reset(timestamp, startingPose);
  }

  /**
   * Update method for the robot state estimator. This method should be called
   * periodically when subsystem is in use.
   * @param timestamp
   */
  public void update(double timestamp) {
    final double left_distance = robot_drive.getLeftLinearPosition() * 12.0;
    final double right_distance = robot_drive.getRightLinearPosition() * 12.0;
    final double delta_left = left_distance - left_encoder_prev_distance;
    final double delta_right = right_distance - right_encoder_prev_distance;
    final Rotation2d gyro_angle = robot_drive.getHeading();
    final Twist2d odometry_velocity = robot_state.generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
    final Twist2d predicted_velocity = Kinematics.forwardKinematics(robot_drive.getLeftLinearVelocity() * 12.0, robot_drive.getRightLinearVelocity() * 12.0);
    robot_state.addObservations(timestamp, odometry_velocity, predicted_velocity);
    left_encoder_prev_distance = left_distance;
    right_encoder_prev_distance = right_distance;
  }

  /**
   * Gets a reference to the robot state object
   * @return the RobotState contained by the RobotStateEstimator subsystem
   */
  public RobotState getRobotState() {
    return robot_state;
  }
}
