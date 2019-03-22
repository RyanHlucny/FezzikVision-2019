package frc.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.planners.DriveMotionPlanner;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Pose2dWithCurvature;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.geometry.Translation2d;
import frc.lib.team254.trajectory.Trajectory;
import frc.lib.team254.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.team254.trajectory.timing.TimedState;
import frc.lib.team254.trajectory.timing.TimingConstraint;

public class TrajectoryGenerator {
    // Constants
    private static final double kMaxVelocity = 150.0; // in/sec
    private static final double kMaxAccel = 70.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 11.5;

    // Private fields
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private TrajectorySet mTrajectorySet = null;
    private DriveMotionPlanner mMotionPlanner;

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    
    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage,
            boolean reverse) {
        return mMotionPlanner.generateTrajectory(waypoints, constraints, max_vel, max_accel, max_voltage, reverse);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage,
            boolean reverse) {
        return mMotionPlanner.generateTrajectory(waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage, reverse);
    }

    // Critical Poses
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d klevel2Start = new Pose2d(22.0, -43.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d klevel1Start = new Pose2d(67.0, -43.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kleaveHab = new Pose2d(110.0, -43.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d klevel1StartMiddle = new Pose2d(67.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Poses for going to back of rocket from start position (Robot drives backwards for these)
    public static final Pose2d kbackRocket1 = new Pose2d(241.0, -107.0, Rotation2d.fromDegrees(-47.0));
    public static final Pose2d kbackRocketFinal = new Pose2d(286.0, -120.0, Rotation2d.fromDegrees(36.0));

    // Pose for going to front of rocket from start position
    public static final Pose2d kfrontRocketFinal = new Pose2d(184.0, -125.0, Rotation2d.fromDegrees(-28.0));
    public static final Pose2d kfrontRocketPlacement = new Pose2d(198.0, -134.0, Rotation2d.fromDegrees(-28.0));
    public static final Pose2d kleaveFrontRocket = new Pose2d(150.0, -134.0, Rotation2d.fromDegrees(0));
    public static final Pose2d kfeederStation = new Pose2d(44.0, -134.0, Rotation2d.fromDegrees(180));

    // Pose for front cargo ship
    public static final Pose2d kfrontCargoShipFinal = new Pose2d(180.0, -11.0, Rotation2d.fromDegrees(0.0));

    // Pose for side cargo ship, left and right slots
    public static final Pose2d ksideCargoShipleft = new Pose2d(260.0, -59.0, Rotation2d.fromDegrees(90.0));
    public static final Pose2d ksideCargoShipMiddle = new Pose2d(281.0, -59.0, Rotation2d.fromDegrees(90.0));

    public class TrajectorySet {
        public class TrajectoryInstance {
            public final Trajectory<TimedState<Pose2dWithCurvature>> trajectory;

            public TrajectoryInstance(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
                this.trajectory = trajectory;
            }
        }

        public final TrajectoryInstance lvl1ToFrontRocket;
        public final TrajectoryInstance frontRocketToFeeder; // back up from rocket
        

        private TrajectorySet() {
            lvl1ToFrontRocket = new TrajectoryInstance(getLvl1ToFrontRocket());
            frontRocketToFeeder = new TrajectoryInstance(getfrontRocketToFeeder());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToFrontRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1Start);
            waypoints.add(kleaveHab);
            waypoints.add(kfrontRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getfrontRocketToFeeder() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfrontRocketPlacement);
            waypoints.add(kleaveFrontRocket);
            waypoints.add(kfeederStation);
            waypoints.add(kfeederStation.transformBy(new Pose2d(-1, 0, Rotation2d.fromDegrees(180))));
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }
    }
}