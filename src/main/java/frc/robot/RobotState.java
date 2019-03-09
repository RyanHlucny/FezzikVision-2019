package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.geometry.Twist2d;
import frc.lib.team254.util.InterpolatingDouble;
import frc.lib.team254.util.InterpolatingTreeMap;

/**
 * An object used for keeping track of the robot's state over time.
 */
public class RobotState {
  private static final int kObservationBufferSize = 100;

  // FPGATimestamp -> RigidTransform2d or Rotation2d
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
  private Twist2d vehicle_velocity_predicted_;
  private Twist2d vehicle_velocity_measured_;
  private double distance_driven_;

  public RobotState() {
      reset(0, new Pose2d());
  }

  /**
   * Generates a field-to-vehicle pose
   * @param x x coordinate to use
   * @param y y coordinate to use
   * @param degrees angle in degrees to use
   * @return generated field-to-vehicle Pose2d
   */
  public static Pose2d generateFieldToVehicle(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }

  /**
   * Resets the field to robot transform (robot's position on the field)
   */
  public void reset(double start_time, Pose2d initial_field_to_vehicle) {
      field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
      field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
      Robot.drive.setHeading(initial_field_to_vehicle.getRotation());
      vehicle_velocity_predicted_ = Twist2d.identity();
      vehicle_velocity_measured_ = Twist2d.identity();
      resetDistanceDriven();
  }

  public void resetDistanceDriven() {
      distance_driven_ = 0.0;
  }

  /**
   * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
   * to fill in the gaps.
   * @return Field to Vehicle pose, or null if there aren't any data records
   */
  public Pose2d getFieldToVehicle(double timestamp) {
      return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
      return field_to_vehicle_.lastEntry();
  }

  public Pose2d getPredictedFieldToVehicle(double lookahead_time) {
      return getLatestFieldToVehicle().getValue()
              .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
  }

  public void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
      field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
  }

  public void addObservations(double timestamp, Twist2d measured_velocity,
                                           Twist2d predicted_velocity) {
      addFieldToVehicleObservation(timestamp,
              Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
      vehicle_velocity_measured_ = measured_velocity;
      vehicle_velocity_predicted_ = predicted_velocity;
  }

  public Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
          right_encoder_delta_distance, Rotation2d current_gyro_angle) {
      final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
      final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
              left_encoder_delta_distance, right_encoder_delta_distance,
              current_gyro_angle);
      distance_driven_ += delta.dx; //do we care about dy here?
      return delta;
  }

  public double getDistanceDriven() {
      return distance_driven_;
  }

  public Twist2d getPredictedVelocity() {
      return vehicle_velocity_predicted_;
  }

  public Twist2d getMeasuredVelocity() {
      return vehicle_velocity_measured_;
  }

  public void outputToSmartDashboard() {
      Pose2d odometry = getLatestFieldToVehicle().getValue();
      SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
      SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
      SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
      SmartDashboard.putNumber("Robot Linear Velocity", vehicle_velocity_measured_.dx);
  }
}