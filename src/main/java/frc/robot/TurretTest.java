// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TurretTest {
    private DCMotor TurretTest;

    private Double kP = 0.0001;

    private Double Kd = 0.0000;
    
    private Double goalX = 0.0;

    private Double lastError = 0.0;

    private Double angleToTolerance = 0.2;

    private Double MAX_POWER = 0.6;

    private double power = 0;

    private final Timer ElapsedTime = new Timer();
}
