package frc.lib.team254.trajectory;

import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
