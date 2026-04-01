// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

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
public class MEIOSIS extends SequentialCommandGroup {
  /** Creates a new test. */
  public MEIOSIS(
    SpindexerSubsystem spindexerSubsystem,
    FeederSubsystem feederSubsystem,
    ShooterSubsystem shooterSubsystem,
    IntakeSubsystem intakeSubsystem,
    IntakePivotSubsystem intakePivotSubsystem,
    HoodSubsystem hoodSubsystem
    ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoPath ME_TR_C = AutoPath.PP("ME_TR_C");
    AutoPath ME_C_BUMP = AutoPath.PP("ME_C_BUMP");
    //AutoPath C_BUMP = AutoPath.PP("C_BUMP");

    addCommands(
      ME_TR_C.resetOdometryToStart(),
      ME_TR_C.follow(),
      ME_C_BUMP.follow(),
      ME_TR_C.follow(),
      ME_C_BUMP.follow()
   );

    
  }
}
