/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.team254.geometry.Pose2d;
import frc.lib.team254.geometry.Rotation2d;
import frc.robot.commands.auto.AutoPathCommand;
import frc.robot.commands.drive.TurnByAngle;
import frc.robot.paths.TrajectoryGenerator;

public class Hab1ToBackRocketWithTurn extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Hab1ToBackRocketWithTurn() {
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
    addSequential(new AutoPathCommand(new Pose2d(67, -43, Rotation2d.fromDegrees(0)), TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToBackRocket));
    addSequential(new TurnByAngle(TrajectoryGenerator.kRightBackRocketTurn));
  }
}
