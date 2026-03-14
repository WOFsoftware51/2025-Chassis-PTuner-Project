// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import java.io.IOException;
import java.text.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.MoveToAngle;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class test extends SequentialCommandGroup {
  /** Creates a new test. */
  public test(Swerve swerve, RobotState robotState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerPath left_Center;

      try {
        left_Center = PathPlannerPath.fromPathFile("Red_Right_Center");

    addCommands(
      AutoBuilder.followPath(left_Center)
    );

      } catch (Exception e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }     

  }
}
