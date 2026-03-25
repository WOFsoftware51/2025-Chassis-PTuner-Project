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
public class BUMP_HP extends SequentialCommandGroup {
  /** Creates a new test. */
  public BUMP_HP(SpindexerSubsystem spindexerSubsystem,FeederSubsystem feederSubsystem,ShooterSubsystem shooterSubsystem,IntakeSubsystem intakeSubsystem,IntakePivotSubsystem intakePivotSubsystem,PivotSubsystem pivotSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoPath TR_HP = AutoPath.PP("TR_HP");
    AutoPath HP_BUMP = AutoPath.PP("HP_BUMP");


    addCommands(
     new ParallelRaceGroup(
       TR_HP.resetOdometryToStart(),
       TR_HP.follow(),
       NamedCommands.getCommand("IntakePivot")),
    HP_BUMP.follow(),
    new ParallelCommandGroup(
      NamedCommands.getCommand("Shoot"),
      NamedCommands.getCommand("Pivot")
    )
    
   );

    
  }
}
