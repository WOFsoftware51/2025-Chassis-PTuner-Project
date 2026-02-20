// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final RobotType robot = Robot.isReal() ? RobotType.ALPHABOT : RobotType.SIMBOT;
  public static final boolean tuningMode = true;

  public static final double loopPeriodSecs = 0.02;
  public static final double loopPeriodWatchdogSecs = 0.2;

  public static Mode getMode() {
    return switch (robot) {
      case COMPBOT, ALPHABOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    COMPBOT,
    ALPHABOT,
    SIMBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static final String kCANIvoreName = "CANivore";

  public static final class IntakeConstants {
    public static final int kMotorID = 40;
  }

  public static final class SpinDexerConstants {
    public static final int kMotorID = 42;
  }
  
  public static final class FeederConstants {
    public static final int kMotorID = 44;
  }
  
  public static final class TurretConstants {
    public static final int kMotorID = 46;

    public static final double kTurretGearRatio = 37.33; //2240:15 (140/15 * 4:1)
    public static final double kForwardLimit = 135.0;
    public static final double kReverseLimit = -135;
        
  }
  
  public static final class PivotConstants {
    public static final int kMotorID = 48;
  }

  public static final class ShooterConstants {
    public static final int kMotorLeft = 50;
    public static final int kMotorRight = 51;
  }

  public static final class HangerConstants {
    public static final double kGearRatio = 381;
  }

  public static final class VisionConstants {
    public static final String kChassisLimelight = "chassis-limelight";
    public static final String kTurretLimelight = "limelight-turret";
  }


}
