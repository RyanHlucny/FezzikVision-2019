package frc.robot.paths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.FrontCargoShipToRightFeeder;
import frc.robot.commands.auto.FrontRocketToFeeder;
import frc.robot.commands.auto.Hab1MiddleToFrontCargoShip;
import frc.robot.commands.auto.Hab1ToBackRocket;
import frc.robot.commands.auto.Hab1ToFrontCargoShip;
import frc.robot.commands.auto.Hab1ToFrontRocket;
import frc.robot.commands.auto.RightFeederToBackRocket;
import frc.robot.commands.auto.RightFeederToFrontRocket;
import frc.robot.commands.auto.commandgroups.BackRocketToFeederWithTurn;
import frc.robot.commands.auto.commandgroups.FrontRocketToFeederWithTurn;
import frc.robot.commands.auto.commandgroups.RightFeederToFrontRocketWithTurn;

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
          m_firstCommand = new Hab1ToFrontRocket();
          m_secondCommand = new FrontRocketToFeederWithTurn();
        }
        else if (dest1 == 1) {
          // back rocket
          m_firstCommand = new Hab1ToBackRocket();
          m_secondCommand = new BackRocketToFeederWithTurn();
        }
        else if (dest1 == 2) {
          // front cargo ship
          m_firstCommand = new Hab1ToFrontCargoShip();
          m_secondCommand = new FrontCargoShipToRightFeeder();
        }
        else {
          // side cargo ship
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
          m_thirdCommand = new RightFeederToFrontRocketWithTurn();
        }
        else if (dest2 == 1) {
          // back rocket
          m_thirdCommand = new RightFeederToBackRocket();
        }
        else if (dest2 == 2) {
          // front cargo ship
        }
        else {
          // side cargo ship
        }
      break;

      // Starting left lvl 1 hab
      case 1:
        // Destination 1:
        if (dest1 == 0) {
          // front rocket
        }
        else if (dest1 == 1) {
          // back rocket
        }
        else if (dest1 == 2) {
          // front cargo ship
        }
        else {
          // side cargo ship
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
        }
        else if (dest2 == 1) {
          // back rocket
        }
        else if (dest2 == 2) {
          // front cargo ship
        }
        else {
          // side cargo ship
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
          m_firstCommand = new Hab1MiddleToFrontCargoShip();
          m_secondCommand = new FrontCargoShipToRightFeeder();
        }
        else {
          // side cargo ship
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
          m_thirdCommand = new RightFeederToFrontRocket();
          m_fourthCommand = new FrontRocketToFeeder();
        }
        else if (dest2 == 1) {
          // back rocket
        }
        else if (dest2 == 2) {
          // front cargo ship
        }
        else {
          // side cargo ship
        }
      break;

      // Starting right lvl 2 hab
      case 3:
      // Destination 1:
      if (dest1 == 0) {
        // front rocket
      }
      else if (dest1 == 1) {
        // back rocket
      }
      else if (dest1 == 2) {
        // front cargo ship
      }
      else {
        // side cargo ship
      }

      // Destination2:
      if (dest2 == 0) {
        // front rocket
      }
      else if (dest2 == 1) {
        // back rocket
      }
      else if (dest2 == 2) {
        // front cargo ship
      }
      else {
        // side cargo ship
      }
      break;

      // Starting left lvl 2 hab
      case 4:
        // Destination 1:
        if (dest1 == 0) {
          // front rocket
        }
        else if (dest1 == 1) {
          // back rocket
        }
        else if (dest1 == 2) {
          // front cargo ship
        }
        else {
          // side cargo ship
        }

        // Destination2:
        if (dest2 == 0) {
          // front rocket
        }
        else if (dest2 == 1) {
          // back rocket
        }
        else if (dest2 == 2) {
          // front cargo ship
        }
        else {
          // side cargo ship
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