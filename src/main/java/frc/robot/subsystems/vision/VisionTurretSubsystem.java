// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class VisionTurretSubsystem extends SubsystemBase {
  private VisionIO io;
  public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Creates a new VisionTurretSubsystem. */
  public VisionTurretSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/Turret Limelight", inputs);
    RobotState.getInstance().setTurretLimelightPose3d(inputs.botpose);
  }
}
