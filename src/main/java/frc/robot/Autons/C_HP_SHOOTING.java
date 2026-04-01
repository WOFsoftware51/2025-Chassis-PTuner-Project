// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class C_HP_SHOOTING extends SequentialCommandGroup {
  /** Creates a new test. */
  public C_HP_SHOOTING(SpindexerSubsystem spindexerSubsystem,FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem,IntakeSubsystem intakeSubsystem,IntakePivotSubsystem intakePivotSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoPath BTW_TR = AutoPath.PP("BTW_TR");
    AutoPath TR_C = AutoPath.PP("TR_C");
    AutoPath C_TR_POS = AutoPath.PP("C_TR_POS");
    AutoPath C_HP = AutoPath.PP("C_HP");

    addCommands(
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
