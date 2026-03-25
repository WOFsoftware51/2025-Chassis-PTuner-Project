// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
public class S_C_S_LEFT extends SequentialCommandGroup {
  /** Creates a new test. */
  public S_C_S_LEFT(SpindexerSubsystem spindexerSubsystem,FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem,IntakeSubsystem intakeSubsystem,IntakePivotSubsystem intakePivotSubsystem, PivotSubsystem pivotSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoPath BTW_TR_LEFT = AutoPath.PP("BTW_TR_LEFT");
    AutoPath TR_C_LEFT = AutoPath.PP("TR_C_LEFT");
    AutoPath C_TR_POS_LEFT = AutoPath.PP("C_TR_POS_LEFT");
    AutoPath C_TR_LEFT = AutoPath.PP("C_TR_LEFT");

    addCommands(
      new ParallelCommandGroup(
        NamedCommands.getCommand("Shoot"),
        NamedCommands.getCommand("Pivot")
        ),
      BTW_TR_LEFT.resetOdometryToStart(),
      BTW_TR_LEFT.follow(),
      NamedCommands.getCommand("IntakePivot"),
      new ParallelRaceGroup(
        TR_C_LEFT.follow(),
        NamedCommands.getCommand("Intake")),
        new ParallelCommandGroup( 
          C_TR_POS_LEFT.follow(),
          NamedCommands.getCommand("IntakePivotUp")),
      C_TR_LEFT.follow(),
      new ParallelCommandGroup(
        NamedCommands.getCommand("ShootMore"),
        NamedCommands.getCommand("PivotMore")
        )
    );

    
  }
}
