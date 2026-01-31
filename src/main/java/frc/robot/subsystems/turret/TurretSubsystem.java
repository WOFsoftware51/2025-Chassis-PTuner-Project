// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private double tx;
  private double currentDegrees;
  private double targetDegrees;
  private double filteredTX;

  private boolean inWindow = false;

  
  public TurretSubsystem(TurretIO io) {
    this.io = io;
  }

  double a = 1;


  private void run(double volts) {
    io.runVolts(Volts.of(volts));
  }

  private void goToSetpoint(double degrees) {
    io.runSetpoint(Degrees.of(degrees));
  }

  private void stop() {
    io.stop();
  }
  
  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-turret");

    tx = table.getEntry("tx").getDouble(0);
    currentDegrees = inputs.position.in(Degrees);
    
    LinearFilter txFilter = LinearFilter.singlePoleIIR(0.1, 0.2);
    filteredTX = txFilter.calculate(tx);
    
    targetDegrees = currentDegrees - filteredTX;
    

    Logger.recordOutput("tx", tx);
    Logger.recordOutput("currentDegrees", currentDegrees);
    Logger.recordOutput("targetDegrees", targetDegrees);
    Logger.recordOutput("inWindow", inWindow);


    this.io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);


    if(Math.abs(tx)>1.0) {
      inWindow = false;
    }
    else if(Math.abs(tx)<0.3) {
      inWindow = true;
    }


  }



  public Command TurretRunWithVolts(double speedInVolts) {
    return run(() ->
      this.run(speedInVolts)
    )
    .finallyDo(
      () -> stop()
    );
  }
  
  public Command TurretToSetpoint(Angle positionInDegrees) {
    return run(
      () ->io.runSetpoint(positionInDegrees)
    )
    .finallyDo(
      () -> stop()
    );
  }


  public Command TurretAimToHub() {
    return runEnd(
      () ->io.runSetpoint(Degrees.of(targetDegrees)), 
      () -> stop()
    )
    .until(
      () -> inWindow
    );
    
    // .finallyDo(
    //   () -> stop()
    // );
  }
  
}
