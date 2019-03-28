/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Pose2dWithCurvature;
import frc.lib.team254.trajectory.TrajectoryIterator;
import frc.lib.team254.trajectory.timing.TimedState;
import frc.robot.Robot;

public class DriveTrajectory extends Command {
  private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> m_trajectory;
  private final Pose2d m_startingPose;


  /**
   * Convenience constructor for creating command that doesn't reset the robot's state estimator. This
   * means that the robot will use its current estimated position as the starting point for the path.
   * @param trajectory the trajectory this command should make the robot follow
   */
  public DriveTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    requires(Robot.drive);
    m_startingPose = null;
    m_trajectory = trajectory;
  }
  
  /**
   * Default constructor
   * @param startingPose the robot's starting pose. Note that if you do set the starting pose yourself, 
   * @param trajectory the trajectory this command should make the robot follow
   * the robot's state estimator will be reset and the reference frame will probably change.
   */
  public DriveTrajectory(Pose2d startingPose, TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    requires(Robot.drive);
    m_startingPose = startingPose;
    m_trajectory = trajectory;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Reset robot state if desired
    if (m_startingPose != null) {
      Robot.stateEstimator.reset(Timer.getFPGATimestamp(), m_startingPose);
    }
    // Set the drive to follow the trajectory
    Robot.drive.setTrajectory(m_trajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Update the robot state
    Robot.stateEstimator.update(Timer.getFPGATimestamp());
    // Update the path follower
    Robot.drive.updatePathFollower();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drive.isDoneWithTrajectory();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
