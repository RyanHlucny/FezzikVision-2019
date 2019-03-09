package frc.team5172.lib.trajectory;

import frc.team5172.lib.geometry.Pose2d;
import frc.team5172.lib.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
