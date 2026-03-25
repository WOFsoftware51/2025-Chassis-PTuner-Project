// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OSMOSIS<FollowPathWithEvents> extends SequentialCommandGroup {
  /** Creates a new test. */

  public OSMOSIS(SpindexerSubsystem spindexerSubsystem,FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem,IntakeSubsystem intakeSubsystem,IntakePivotSubsystem intakePivotSubsystem,PivotSubsystem pivotSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
//     AutoPath OS_TR_C = AutoPath.PP("OS_TR_C");
//     AutoPath OS_C_BUMP = AutoPath.PP("OS_C_BUMP");
    
// addCommands(
//   NamedCommands.getCommand("IntakePivot"),
//   OS_TR_C.resetOdometryToStart(),
//   OS_TR_C.follow().alongWith(NamedCommands.getCommand("intake")),
//   NamedCommands.getCommand("IntakePivotUp"),
//   OS_C_BUMP.follow().alongWith( new ParallelRaceGroup(
//     NamedCommands.getCommand("Shoot").alongWith(NamedCommands.getCommand("Pivot")))),
//     NamedCommands.getCommand("IntakePivot"),
//   OS_TR_C.follow().alongWith(NamedCommands.getCommand("intake")),
//   OS_C_BUMP.follow().alongWith( new ParallelRaceGroup(
//     NamedCommands.getCommand("Shoot").alongWith(NamedCommands.getCommand("Pivot"))))
// );
  

  }
}
