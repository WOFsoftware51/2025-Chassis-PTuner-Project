// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class S_C_S extends SequentialCommandGroup {
  /** Creates a new test. */
  public S_C_S(
    SpindexerSubsystem spindexerSubsystem,
    FeederSubsystem feederSubsystem,
    ShooterSubsystem shooterSubsystem,
    IntakeSubsystem intakeSubsystem,
    IntakePivotSubsystem intakePivotSubsystem,
    HoodSubsystem hoodSubsystem
    ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoPath TR_C = AutoPath.PP("TR_C");
    AutoPath C_TR_POS = AutoPath.PP("C_TR_POS");
    AutoPath C_TR = AutoPath.PP("C_TR");

    addCommands(
      TR_C.resetOdometryToStart(),
      TR_C.follow(),
      C_TR_POS.follow(),
      C_TR.follow(),
      NamedCommands.getCommand("Score")
    //   BTW_BUMP.resetOdometryToStart(),
    //   new ParallelCommandGroup(
    //     NamedCommands.getCommand("Shoot"),
    //     NamedCommands.getCommand("Pivot")
    //    ),
    //    new ParallelCommandGroup(
    //    BTW_BUMP.follow(),
    //    NamedCommands.getCommand("IntakePivot")
    //    ),
    //  new ParallelRaceGroup(
    //     BUMP_C.follow(),
    //     NamedCommands.getCommand("Intake")),
    //     new ParallelCommandGroup( 
    //       C_BUMP.follow(),
    //       NamedCommands.getCommand("IntakePivotUp")),
    //   new ParallelCommandGroup(
    //     NamedCommands.getCommand("ShootMore"),
    //     NamedCommands.getCommand("PivotMore")
    //     )
    );

    
  }
}
