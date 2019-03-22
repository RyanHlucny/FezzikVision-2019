package frc.robot.planners;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Pose2dWithCurvature;
import frc.lib.team254.physics.DCMotorTransmission;
import frc.lib.team254.physics.DifferentialDrive;
import frc.lib.team254.trajectory.DistanceView;
import frc.lib.team254.trajectory.Trajectory;
import frc.lib.team254.trajectory.TrajectoryIterator;
import frc.lib.team254.trajectory.TrajectorySamplePoint;
import frc.lib.team254.trajectory.TrajectoryUtil;
import frc.lib.team254.trajectory.timing.DifferentialDriveDynamicsConstraint;
import frc.lib.team254.trajectory.timing.TimedState;
import frc.lib.team254.trajectory.timing.TimingConstraint;
import frc.lib.team254.trajectory.timing.TimingUtil;
import frc.lib.team254.util.Units;
import frc.lib.team254.util.Util;

public class DriveMotionPlanner {
    // Constants
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    // private fields
    private final DifferentialDrive mModel;
    private TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    private double mLastTime = Double.POSITIVE_INFINITY;
    private TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    private Pose2d mError = Pose2d.identity();
    private Output mOutput = new Output();
    private DifferentialDrive.ChassisState mPrevVelocity = new DifferentialDrive.ChassisState();
    private double mDt = 0.0;

    public DriveMotionPlanner() {
        final DCMotorTransmission transmission = new DCMotorTransmission(
            Constants.DRIVE_Kv,
            Units.inches_to_meters(Constants.WHEEL_RADIUS_INCHES) * Units.inches_to_meters(Constants.WHEEL_RADIUS_INCHES) * Constants.ROBOT_LINEAR_INERTIA * 2.0 * Constants.DRIVE_Ka,
            Constants.DRIVE_V_INTERCEPT);
        mModel = new DifferentialDrive(
            Constants.ROBOT_LINEAR_INERTIA, 
            Constants.ROBOT_ANGULAR_INERTIA, 
            Constants.ROBOT_ANGULAR_DRAG, 
            Units.inches_to_meters(Constants.WHEEL_RADIUS_INCHES),
            Units.inches_to_meters(Constants.WHEEL_BASE_INCHES),
            transmission, transmission);
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = new Output();
        mLastTime = Double.POSITIVE_INFINITY;
    }

    /**
     * Convenience method for generating trajectories
     * @param waypoints list of waypoints to use to generate the trajectory
     * @param constraints list of constraints to apply to the trajectory
     * @param max_vel maximum allowable robot velocity
     * @param max_accel maximum allowable robot acceleration
     * @param max_voltage maximum allowable voltage to the robot's motors
     * @return The generated trajectory
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,
            double max_accel,
            double max_voltage) {

        return generateTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage, false);
    }

    /**
     * Convenience method for generating trajectories
     * @param waypoints list of waypoints to use to generate the trajectory
     * @param constraints list of constraints to apply to the trajectory
     * @param max_vel maximum allowable robot velocity
     * @param max_accel maximum allowable robot acceleration
     * @param max_voltage maximum allowable voltage to the robot's motors
     * @param reverse whether or not the robot should drive the trajectory in reverse
     * @return The generated trajectory
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,
            double max_accel,
            double max_voltage,
            boolean reverse) {

        return generateTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage, reverse);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_accel,
            double max_voltage,
            boolean reverse) {

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypoints, kMaxDx, kMaxDy, kMaxDTheta);

        // Create the constraint that the robot must be able to traverse the trajectory without applying more
        // than the maximum voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new DifferentialDriveDynamicsConstraint<>(mModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        
        // Generate the timed trajectory
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(
            reverse, new DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
        
        return timed_trajectory;
    }

    public Output update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new Output();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
        mSetpoint = sample_point.state();

        // If path isn't finished, update nonlinear feedback controller
        if (!mCurrentTrajectory.isDone()) {
            final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
            final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
            final double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.state().getDCurvatureDs()));
            final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m), 
                new DifferentialDrive.ChassisState(acceleration_m, acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
            // Update nonlinear feedback controller
            mOutput = updateNonlinearFeedback(dynamics, current_state);
        }
        else {
            mOutput = new Output();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
    
    protected Output updateNonlinearFeedback(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        // Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        final double kBeta = 2.0; // >0
        final double kZeta = 0.7; // Damping coefficient, [0, 1]

        // Compute gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear
            + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);
        
        // Compute error components.
        final double angle_error_rads = mError.getRotation().getRadians();
        final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ? 1.0 : mError.getRotation().sin() / angle_error_rads;
        final DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState(
            dynamics.chassis_velocity.linear * mError.getRotation().cos() + k * Units.inches_to_meters(mError.getTranslation().x()),
            dynamics.chassis_velocity.angular + k * angle_error_rads + dynamics.chassis_velocity.linear * kBeta * sin_x_over_x * Units.inches_to_meters(mError.getTranslation().y()));
        
        // Compute adjusted left and right wheel velocities.
        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

        dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.linear - mPrevVelocity.linear) / mDt;
        dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.angular - mPrevVelocity.angular) / mDt;

        mPrevVelocity = dynamics.chassis_velocity;

        DifferentialDrive.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
            dynamics.chassis_acceleration).voltage;
        
        return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration.left,
            dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
    }

    public static class Output {
        public Output() {
        }

        public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
                      double left_feedforward_voltage, double
                              right_feedforward_voltage) {
            this.left_velocity = left_velocity;
            this.right_velocity = right_velocity;
            this.left_accel = left_accel;
            this.right_accel = right_accel;
            this.left_feedforward_voltage = left_feedforward_voltage;
            this.right_feedforward_voltage = right_feedforward_voltage;
        }

        public double left_velocity;  // rad/s
        public double right_velocity;  // rad/s

        public double left_accel;  // rad/s^2
        public double right_accel;  // rad/s^2

        public double left_feedforward_voltage;
        public double right_feedforward_voltage;

        public void flip() {
            double tmp_left_velocity = left_velocity;
            left_velocity = -right_velocity;
            right_velocity = -tmp_left_velocity;

            double tmp_left_accel = left_accel;
            left_accel = -right_accel;
            right_accel = -tmp_left_accel;

            double tmp_left_feedforward = left_feedforward_voltage;
            left_feedforward_voltage = -right_feedforward_voltage;
            right_feedforward_voltage = -tmp_left_feedforward;
        }
    }
}