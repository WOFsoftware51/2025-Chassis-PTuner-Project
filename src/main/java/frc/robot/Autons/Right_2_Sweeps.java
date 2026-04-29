// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import java.nio.file.Paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths10;
import frc.robot.RobotState;
import frc.robot.commands.factories.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right_2_Sweeps extends SequentialCommandGroup {
  /** Creates a new Right_Middle2Cycle. */
  public Right_2_Sweeps(
      Swerve swerve, 
      RobotState robotState, 
      ShooterSubsystem shooter, 
      TurretSubsystem turret, 
      IntakePivotSubsystem intakePivot,
      IntakeSubsystem intake, 
      FeederSubsystem feeder, 
      SpindexerSubsystem spindexer, 
      HoodSubsystem hood, 
      Superstructure superstructure,
      Paths10 path
  ) 
  {

    try {
      addCommands(
        AutoBuilder.resetOdom(path.Right_2_Sweeps.getStartingHolonomicPose().get()),
        Commands.parallel(
          AutoBuilder.followPath(path.emptyRightTrench),
          shooter.runRPMCommand(3000).withTimeout(0.05)
        ),
        Commands.race(
          AutoBuilder.followPath(path.Right_2_Sweeps), //go to center
          Commands.sequence(
            Commands.waitSeconds(0.5),
            intakePivot.goDown(), 
            intake.runVolts(10.8)
          )
        ),
        Commands.race( //shoot
          superstructure.shoot(),
          Commands.run(() -> turret.turretCameraAimToHub()),
          Commands.sequence(
            Commands.waitSeconds(3), 
            intakePivot.bounce().alongWith(intake.runVolts(6))
          ),
          Commands.waitSeconds(20)
        ))
          ;  
      

    }
    catch(Exception e) {
      e.printStackTrace();
    }  

  }
}
