// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.MoveToAngle;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends SequentialCommandGroup {
  /** Creates a new test. */
  public Test(Swerve swerve, RobotState robotState, ShooterSubsystem shooter, FeederSubsystem feeder, SpindexerSubsystem spindexer, PivotSubsystem pivot) {

    // addCommands(
    //   new MoveToAngle(
    //     swerve, 
    //     robotState, 
    //     robotState.getPose2d(),
    //     () -> robotState.justinTurretAngle(),
    //     // () -> robotState.getRobotToAllianceHubDegrees(),
    //     // () -> robotState.getTurretToAllianceHubDegrees(),
    //     1
    //   ), 
    // //   shooter.treeMapRPMCommand().raceWith(pivot.treeMapRPMCommand()).until(() -> shooter.atRPM), 
    //   feeder.runFeederVoltsCommand(12).alongWith(spindexer.runSpindexerVoltsCommand(12)).alongWith(pivot.treeMapRPMCommand()).alongWith(shooter.treeMapRPMCommand()).alongWith(pivot.treeMapRPMCommand())



    // );
  }
}
