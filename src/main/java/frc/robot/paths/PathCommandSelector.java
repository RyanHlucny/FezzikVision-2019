package frc.robot.paths;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Rotation2d;
import frc.lib.team254.trajectory.TimedView;
import frc.lib.team254.trajectory.TrajectoryIterator;
import frc.robot.commands.auto.DriveTrajectory;
import frc.robot.commands.auto.commandgroups.BackRocketToFeederWithTurn;
import frc.robot.commands.auto.commandgroups.FrontCargoShipToRightFeederWithTurn;
import frc.robot.commands.auto.commandgroups.FrontRocketToFeederWithTurn;
import frc.robot.commands.auto.commandgroups.Hab1ToSideCargoShipMiddleWithTurn;
import frc.robot.commands.auto.commandgroups.Hab2ToBackRocketWithTurn;
import frc.robot.commands.auto.commandgroups.Hab2ToSideCargoShipMiddleWithTurn;
import frc.robot.commands.auto.commandgroups.RightFeederToBackRocketWithTurn;
import frc.robot.commands.auto.commandgroups.RightFeederToFrontCargoShipWithTurn;
import frc.robot.commands.auto.commandgroups.RightFeederToFrontRocketWithTurn;
import frc.robot.commands.auto.commandgroups.RightFeederToSideCargoShipLeftWithTurn;
import frc.robot.commands.auto.commandgroups.SideCargoShipToRightFeederWithTurn;

/**
 * This class selects the appropriate path following command for each phase of
 * the sandstorm semi-autonomous routine. It is meant to be used by the
 * sandstorm state machine in the main Robot class.
 */
public class PathCommandSelector {
  // Private fields
  private Command m_firstCommand = null;
  private Command m_secondCommand = null;
  private Command m_thirdCommand = null;
  private Command m_fourthCommand = null;

  private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

  /**
   * Public constructor
   * @param startPosition index of the starting position of the robot.
   *    0 - right lvl 1 hab
   *    1 - left lvl 1 hab
   *    2 - middle lvl 1 hab
   *    3 - right lvl 2 hab
   *    4 - left lvl 2 hab
   * @param dest1 index of the 1st destination.
   *    0 - front rocket
   *    1 - back rocket
   *    2 - front cargo ship
   *    3 - side cargo ship
   * @param dest2 index of the 2nd destination.
   *    0 - front rocket
   *    1 - back rocket
   *    2 - front cargo ship
   *    3 - side cargo ship
   */
  public PathCommandSelector(int startPosition, int dest1, int dest2) {
    // Select the correct path commands and their corresponding feeder returns
    switch(startPosition) {
      // starting right lvl 1 hab
      case 0:
        // Destination 1:
        if (dest1 == 0) {
          // front rocket
          m_firstCommand = new DriveTrajectory(new Pose2d(67, -43, Rotation2d.fromDegrees(180)), new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToFrontRocket.right))); // Hab1ToFrontRocket
          m_secondCommand = new FrontRocketToFeederWithTurn(true);
        }
        else if (dest1 == 1) {
          // back rocket
          m_firstCommand = new DriveTrajectory(new Pose2d(67, -43, Rotation2d.fromDegrees(0)), new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToBackRocket.right))); // Hab1ToBackRocket
          m_secondCommand = new BackRocketToFeederWithTurn(true);
        }
        else if (dest1 == 2) {
          // front cargo ship
          m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel1StartReverse, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToFrontCargoship.right))); //Hab1ToFrontCargoShip
          m_secondCommand = new FrontCargoShipToRightFeederWithTurn(true);
        }
        else {
          // side cargo ship
          m_firstCommand = new Hab1ToSideCargoShipMiddleWithTurn(true);
          m_secondCommand = new SideCargoShipToRightFeederWithTurn(true);
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
          m_thirdCommand = new RightFeederToFrontRocketWithTurn(true);
          m_fourthCommand = new FrontRocketToFeederWithTurn(true);
        }
        else if (dest2 == 1) {
          // back rocket
          m_thirdCommand = new RightFeederToBackRocketWithTurn(true);
          m_fourthCommand = new BackRocketToFeederWithTurn(true);
        }
        else if (dest2 == 2) {
          // front cargo ship
          m_thirdCommand = new RightFeederToFrontCargoShipWithTurn(true);
          m_fourthCommand = new FrontCargoShipToRightFeederWithTurn(true);
        }
        else {
          // side cargo ship
          m_thirdCommand = new RightFeederToSideCargoShipLeftWithTurn(true);
          m_fourthCommand = new SideCargoShipToRightFeederWithTurn(true);
        }
      break;

      // Starting left lvl 1 hab
      case 1:
        // Destination 1:
        if (dest1 == 0) {
          // front rocket
          m_firstCommand = new DriveTrajectory(new Pose2d(67, -43, Rotation2d.fromDegrees(180)), new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToFrontRocket.left))); // Hab1ToFrontRocket
          m_secondCommand = new FrontRocketToFeederWithTurn(false);
        }
        else if (dest1 == 1) {
          // back rocket
          m_firstCommand = new DriveTrajectory(new Pose2d(67, -43, Rotation2d.fromDegrees(0)), new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToBackRocket.left))); // Hab1ToBackRocket
          m_secondCommand = new BackRocketToFeederWithTurn(false);
        }
        else if (dest1 == 2) {
          // front cargo ship
          m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel1StartReverse, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToFrontCargoship.left))); //Hab1ToFrontCargoShip
          m_secondCommand = new FrontCargoShipToRightFeederWithTurn(false);
        }
        else {
          // side cargo ship
          m_firstCommand = new Hab1ToSideCargoShipMiddleWithTurn(false);
          m_secondCommand = new SideCargoShipToRightFeederWithTurn(false);
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
          m_thirdCommand = new RightFeederToFrontRocketWithTurn(false);
          m_fourthCommand = new FrontRocketToFeederWithTurn(false);
        }
        else if (dest2 == 1) {
          // back rocket
          m_thirdCommand = new RightFeederToBackRocketWithTurn(false);
          m_fourthCommand = new BackRocketToFeederWithTurn(false);
        }
        else if (dest2 == 2) {
          // front cargo ship
          m_thirdCommand = new RightFeederToFrontCargoShipWithTurn(false);
          m_fourthCommand = new FrontCargoShipToRightFeederWithTurn(false);
        }
        else {
          // side cargo ship
          m_thirdCommand = new RightFeederToSideCargoShipLeftWithTurn(false);
          m_fourthCommand = new SideCargoShipToRightFeederWithTurn(false);
        }
      break;

      // Starting middle lvl 1 hab
      case 2:
        // Destination 1:
        if (dest1 == 0) {
          // front rocket
        }
        else if (dest1 == 1) {
          // back rocket
        }
        else if (dest1 == 2) {
          // front cargo ship
          m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel1StartMiddle, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().midToFrontCargoship.right)));
          m_secondCommand = new FrontCargoShipToRightFeederWithTurn(true);
        }
        else {
          // side cargo ship
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
          m_thirdCommand = new RightFeederToFrontRocketWithTurn(true);
          m_fourthCommand = new FrontRocketToFeederWithTurn(true);
        }
        else if (dest2 == 1) {
          // back rocket
          m_thirdCommand = new RightFeederToBackRocketWithTurn(true);
          m_fourthCommand = new BackRocketToFeederWithTurn(true);
        }
        else if (dest2 == 2) {
          // front cargo ship
          m_thirdCommand = new RightFeederToFrontCargoShipWithTurn(true);
          m_fourthCommand = new FrontCargoShipToRightFeederWithTurn(true);
        }
        else {
          // side cargo ship
          m_thirdCommand = new RightFeederToSideCargoShipLeftWithTurn(true);
          m_fourthCommand = new SideCargoShipToRightFeederWithTurn(true);
        }
      break;

      // Starting right lvl 2 hab
      case 3:
      // Destination 1:
      if (dest1 == 0) {
        // front rocket
        m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel2StartReverse, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl2ToFrontRocket.right)));
        m_secondCommand = new FrontRocketToFeederWithTurn(true);
      }
      else if (dest1 == 1) {
        // back rocket
        m_firstCommand = new Hab2ToBackRocketWithTurn(true);
        m_secondCommand = new BackRocketToFeederWithTurn(true);
      }
      else if (dest1 == 2) {
        // front cargo ship
        m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel2StartReverse, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl2ToFrontCargoShip.right)));
        m_secondCommand = new FrontCargoShipToRightFeederWithTurn(true);
      }
      else {
        // side cargo ship
        m_firstCommand = new Hab2ToSideCargoShipMiddleWithTurn(true);
        m_secondCommand = new SideCargoShipToRightFeederWithTurn(true);
      }

      // Destination2:
      if (dest2 == 0) {
        // front rocket
        m_thirdCommand = new RightFeederToFrontRocketWithTurn(true);
        m_fourthCommand = new FrontRocketToFeederWithTurn(true);
      }
      else if (dest2 == 1) {
        // back rocket
        m_thirdCommand = new RightFeederToBackRocketWithTurn(true);
        m_fourthCommand = new BackRocketToFeederWithTurn(true);
      }
      else if (dest2 == 2) {
        // front cargo ship
        m_thirdCommand = new RightFeederToFrontCargoShipWithTurn(true);
        m_fourthCommand = new FrontCargoShipToRightFeederWithTurn(true);
      }
      else {
        // side cargo ship
        m_thirdCommand = new RightFeederToSideCargoShipLeftWithTurn(true);
        m_fourthCommand = new SideCargoShipToRightFeederWithTurn(true);
      }
      break;

      // Starting left lvl 2 hab
      case 4:
        // Destination 1:
        if (dest1 == 0) {
          // front rocket
          m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel2StartReverse, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl2ToFrontRocket.left)));
          m_secondCommand = new FrontRocketToFeederWithTurn(false);
        }
        else if (dest1 == 1) {
          // back rocket
          m_firstCommand = new Hab2ToBackRocketWithTurn(false);
          m_secondCommand = new BackRocketToFeederWithTurn(false);
        }
        else if (dest1 == 2) {
          // front cargo ship
          m_firstCommand = new DriveTrajectory(TrajectoryGenerator.klevel2StartReverse, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl2ToFrontCargoShip.left)));
          m_secondCommand = new FrontCargoShipToRightFeederWithTurn(false);
        }
        else {
          // side cargo ship
          m_firstCommand = new Hab2ToSideCargoShipMiddleWithTurn(false);
          m_secondCommand = new SideCargoShipToRightFeederWithTurn(false);
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
          m_thirdCommand = new RightFeederToFrontRocketWithTurn(false);
          m_fourthCommand = new FrontRocketToFeederWithTurn(false);
        }
        else if (dest2 == 1) {
          // back rocket
          m_thirdCommand = new RightFeederToBackRocketWithTurn(false);
          m_fourthCommand = new BackRocketToFeederWithTurn(false);
        }
        else if (dest2 == 2) {
          // front cargo ship
          m_thirdCommand = new RightFeederToFrontCargoShipWithTurn(false);
          m_fourthCommand = new FrontCargoShipToRightFeederWithTurn(false);
        }
        else {
          // side cargo ship
          m_thirdCommand = new RightFeederToSideCargoShipLeftWithTurn(false);
          m_fourthCommand = new SideCargoShipToRightFeederWithTurn(false);
        }
      break;

      default:
      break;
    }
  }

  /**
   * Gets the first path command for the sandstorm
   * @return the first path command
   */
  public Command getFirstPathCommand() {
    return m_firstCommand;
  }

  public Command getSecondPathCommand() {
    return m_secondCommand;
  }

  public Command getThirdPathCommand() {
    return m_thirdCommand;
  }

  public Command getFourthPathCommand() {
    return m_fourthCommand;
  }
}