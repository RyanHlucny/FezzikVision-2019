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
import frc.lib.team254.trajectory.TimedView;
import frc.lib.team254.trajectory.TrajectoryIterator;
import frc.lib.team254.trajectory.timing.TimedState;
import frc.robot.Robot;
import frc.robot.paths.TrajectoryGenerator.TrajectorySet.TrajectoryInstance;

public class AutoPathCommand extends Command {
  private Pose2d pose;
  private TrajectoryInstance trajectoryInstance;

  public AutoPathCommand(Pose2d startPose, TrajectoryInstance path) {
    requires(Robot.drive);
    this.pose = startPose;
    this.trajectoryInstance = path;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Reset robot state
    Robot.stateEstimator.reset(Timer.getFPGATimestamp(), pose);
    // Get generated trajectory
    TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory = new TrajectoryIterator<>(new TimedView<>(trajectoryInstance.trajectory));
    // Set the drive to follow the trajectory
    Robot.drive.setTrajectory(trajectory);
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
