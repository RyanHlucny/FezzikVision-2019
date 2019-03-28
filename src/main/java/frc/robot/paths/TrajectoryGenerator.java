package frc.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.planners.DriveMotionPlanner;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Pose2dWithCurvature;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.trajectory.Trajectory;
import frc.lib.team254.trajectory.TrajectoryUtil;
import frc.lib.team254.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.team254.trajectory.timing.TimedState;
import frc.lib.team254.trajectory.timing.TimingConstraint;

public class TrajectoryGenerator {
    // Constants
    private static final double kMaxVelocity = 100.0; // in/sec
    private static final double kMaxAccel = 100.0;
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
    public static final Pose2d klevel2StartReverse = new Pose2d(22.0, -43.0, Rotation2d.fromDegrees(180.0)); // 30 was 22
    public static final Pose2d klevel1StartReverse = new Pose2d(67.0, -43.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d klevel2StartForward = new Pose2d(30.0, -43.0, Rotation2d.fromDegrees(0.0)); // 30 was 22
    public static final Pose2d klevel1StartForward = new Pose2d(67.0, -43.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kleaveHabRight = new Pose2d(110.0, -43.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kleaveHabMiddle = new Pose2d(110.0, 0.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d klevel1StartMiddle = new Pose2d(67.0, 0.0, Rotation2d.fromDegrees(180.0));

    // Poses for going to back of rocket from start position (Robot drives backwards for these)
    public static final Pose2d kbackRocketFinal = new Pose2d(277.0, -124.0, Rotation2d.fromDegrees(-30.0));
    public static final Pose2d kbackRocketPlacement = new Pose2d(277.0, -124.0, Rotation2d.fromDegrees(30.0)); // after backing up and turning -58 degrees
    public static final Pose2d kbackRocket1 = new Pose2d(291.0, -116.0, Rotation2d.fromDegrees(30)); // back away from rocket after placement
    public static final Pose2d kbackRocket2 = new Pose2d(291.0, -116.0, Rotation2d.fromDegrees(170));

    // Pose for going to front of rocket from start position
    public static final Pose2d kfrontRocketFinal = new Pose2d(176.0, -121.0, Rotation2d.fromDegrees(152.0));
    public static final Pose2d kfrontRocketPlacement = new Pose2d(198.0, -134.0, Rotation2d.fromDegrees(152.0));
    public static final Pose2d kleaveFrontRocket = new Pose2d(150.0, -124.0, Rotation2d.fromDegrees(180));
    public static final Pose2d kfeederStation = new Pose2d(60.0, -134.0, Rotation2d.fromDegrees(180));
    public static final Pose2d kfeederStationGrab = new Pose2d(27, -134, Rotation2d.fromDegrees(0));

    // Pose for front cargo ship
    public static final Pose2d kfrontCargoShipRightFinal = new Pose2d(165.0, -11.0, Rotation2d.fromDegrees(180.0)); // Going to right front cargoship from right starting position
    public static final Pose2d kfrontCargoShipLeftFinal = new Pose2d(165.0, 11.0, Rotation2d.fromDegrees(90.0)); // Going to left front cargoship from right feeder

    // Pose for side cargo ship, left and right slots
    public static final Pose2d ksideCargoShipLeftReverse = new Pose2d(260.0, -59.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d ksideCargoShipMiddleReverse = new Pose2d(281.0, -59.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d ksideCargoShipLeftForward = new Pose2d(260.0, -59.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d ksideCargoShipMiddleForward = new Pose2d(281.0, -59.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d ksideCargoShipLeftPlacement = new Pose2d(260.0, -50.0, Rotation2d.fromDegrees(270.0));
    public static final Pose2d ksideCargoShipMiddlePlacement = new Pose2d(281.0, -50.0, Rotation2d.fromDegrees(270.0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public final Trajectory<TimedState<Pose2dWithCurvature>> right; 
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }
        }

        // Right lvl 1 trajectories
        public final MirroredTrajectory lvl1ToFrontRocket;
        public final MirroredTrajectory lvl1ToBackRocket;
        public final MirroredTrajectory lvl1ToFrontCargoship;
        public final MirroredTrajectory lvl1ToSideCargoshipLeft; // from right lvl 1 to left side cargoship
        public final MirroredTrajectory lvl1ToSideCargoshipMiddle; // from right lvl 1 to middle side cargoship
        // Right lvl 2 trajectories
        public final MirroredTrajectory lvl2ToFrontRocket;
        public final MirroredTrajectory lvl2ToBackRocket;
        public final MirroredTrajectory lvl2ToFrontCargoShip;
        public final MirroredTrajectory lvl2ToSideCargoshipLeft;
        public final MirroredTrajectory lvl2ToSideCargoshipMiddle; 
        // Middle lvl 1 trajectories
        public final MirroredTrajectory midToFrontCargoship;
        // Field element to right feeder trajectories
        public final MirroredTrajectory frontRocketToFeeder; // back up from front rocket
        public final MirroredTrajectory backRocketToFeeder; // back up from back rocket
        public final MirroredTrajectory frontCargoShipToRightFeeder; // back from front right cargoship to right feeder station
        public final MirroredTrajectory sideCargoShipToRightFeeder; // back from middle side cargship to right feeder station
        // Right feeder to field element trajectories
        public final MirroredTrajectory rightFeederToFrontRocket;
        public final MirroredTrajectory rightFeederToBackRocket;
        public final MirroredTrajectory rightFeederToFrontCargoship;
        public final MirroredTrajectory rightFeederToSideCargoshipLeft; // right feeder to left side cargoship
        public final MirroredTrajectory rightFeederToSideCargoshipMiddle; // right feeder to middle side cargoship
        // Misc. trajectories
        public final MirroredTrajectory backRocketBackup;
        

        private TrajectorySet() {
            lvl1ToFrontRocket = new MirroredTrajectory(getLvl1ToFrontRocket());
            frontRocketToFeeder = new MirroredTrajectory(getfrontRocketToFeeder());
            midToFrontCargoship = new MirroredTrajectory(getMiddleHabToFrontCargoShip());
            frontCargoShipToRightFeeder = new MirroredTrajectory(getFrontCargoshipToRightFeeder());
            rightFeederToFrontRocket = new MirroredTrajectory(getRightFeederToFrontRocket());
            rightFeederToBackRocket = new MirroredTrajectory(getRightFeederToBackRocket());
            lvl1ToBackRocket = new MirroredTrajectory(getLvl1ToBackRocket());
            backRocketToFeeder = new MirroredTrajectory(getbackRocketToFeeder());
            lvl1ToFrontCargoship = new MirroredTrajectory(getlvl1ToFrontCargoship());
            rightFeederToFrontCargoship = new MirroredTrajectory(getRightFeederToFrontCargoship());
            lvl1ToSideCargoshipLeft = new MirroredTrajectory(getLvl1ToSideCargoshipLeft());
            lvl1ToSideCargoshipMiddle = new MirroredTrajectory(getLvl1ToSideCargoshipMiddle());
            sideCargoShipToRightFeeder = new MirroredTrajectory(getSideCargoshipToRightFeeder());
            rightFeederToSideCargoshipLeft = new MirroredTrajectory(getRightFeederToSideCargoshipLeft());
            rightFeederToSideCargoshipMiddle = new MirroredTrajectory(getRightFeederToSideCargoshipMiddle());
            lvl2ToSideCargoshipLeft = new MirroredTrajectory(getLvl2ToSideCargoshipLeft());
            lvl2ToSideCargoshipMiddle = new MirroredTrajectory(getLvl2ToSideCargoshipMiddle());
            lvl2ToFrontCargoShip = new MirroredTrajectory(getlvl2ToFrontCargoship());
            lvl2ToFrontRocket = new MirroredTrajectory(getLvl2ToFrontRocket());
            lvl2ToBackRocket = new MirroredTrajectory(getLvl2ToBackRocket()); 
            backRocketBackup = new MirroredTrajectory(getBackRocketBackup());
            
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToFrontRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1StartReverse);
            waypoints.add(kfrontRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1StartForward);
            waypoints.add(kbackRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToSideCargoshipMiddle() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1StartReverse);
            waypoints.add(ksideCargoShipMiddleReverse);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToSideCargoshipLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1StartReverse);
            waypoints.add(ksideCargoShipLeftReverse);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl2ToSideCargoshipLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel2StartReverse);
            waypoints.add(ksideCargoShipLeftReverse);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl2ToSideCargoshipMiddle() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel2StartReverse);
            waypoints.add(ksideCargoShipMiddleReverse);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl2ToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel2StartForward);
            waypoints.add(kbackRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl2ToFrontRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel2StartReverse);
            waypoints.add(kfrontRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getlvl2ToFrontCargoship() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel2StartReverse);
            waypoints.add(kfrontCargoShipRightFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                50, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getfrontRocketToFeeder() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfrontRocketPlacement);
            waypoints.add(kleaveFrontRocket);
            waypoints.add(kfeederStation);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getbackRocketToFeeder() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(290.0, -115.0, Rotation2d.fromDegrees(-10)));
            waypoints.add(new Pose2d(60.0, -134.0, Rotation2d.fromDegrees(0)));
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleHabToFrontCargoShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1StartMiddle);
            waypoints.add(kleaveHabMiddle);
            waypoints.add(kfrontCargoShipRightFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                50, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getlvl1ToFrontCargoship() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(klevel1StartReverse);
            waypoints.add(kfrontCargoShipRightFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                50, kMaxAccel, kMaxVoltage, true);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFrontCargoshipToRightFeeder() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(190, -10, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(126, -83, Rotation2d.fromDegrees(-90)));
            waypoints.add(kfeederStation);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideCargoshipToRightFeeder() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(ksideCargoShipMiddlePlacement);
            waypoints.add(new Pose2d(150, -115, Rotation2d.fromDegrees(200)));
            waypoints.add(kfeederStation);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFeederToFrontRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfeederStationGrab);
            waypoints.add(kfrontRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFeederToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfeederStationGrab);
            waypoints.add(new Pose2d(228, -107, Rotation2d.fromDegrees(0)));
            waypoints.add(kbackRocketFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFeederToFrontCargoship() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfeederStationGrab);
            waypoints.add(kfrontCargoShipLeftFinal);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFeederToSideCargoshipLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfeederStationGrab);
            waypoints.add(ksideCargoShipLeftForward);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightFeederToSideCargoshipMiddle() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kfeederStationGrab);
            waypoints.add(ksideCargoShipMiddleForward);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackRocketBackup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kbackRocketPlacement);
            waypoints.add(kbackRocket1);
            return generateTrajectory(waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage, false);
        }
    }
}