/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.team254.trajectory.TimedView;
import frc.lib.team254.trajectory.TrajectoryIterator;
import frc.robot.commands.auto.DriveTrajectory;
import frc.robot.commands.drive.TurnByAngle;
import frc.robot.paths.TrajectoryGenerator;

public class Hab2ToBackRocketWithTurn extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Hab2ToBackRocketWithTurn(boolean isRight) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    if (isRight) {
      addSequential(new DriveTrajectory(TrajectoryGenerator.klevel2StartForward, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl2ToBackRocket.right))));
      addSequential(new TurnByAngle(TrajectoryGenerator.kRightBackRocketTurn));
    }
    else {
      addSequential(new DriveTrajectory(TrajectoryGenerator.klevel2StartForward, new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance().getTrajectorySet().lvl2ToBackRocket.left))));
      addSequential(new TurnByAngle(-TrajectoryGenerator.kRightBackRocketTurn));
    }
  }
}
