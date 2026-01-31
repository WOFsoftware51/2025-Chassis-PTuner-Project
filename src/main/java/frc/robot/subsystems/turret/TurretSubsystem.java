// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

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
    this.io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

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

  public Command TurretAimToHub(Angle tx) {
    return run(
      () ->io.runSetpoint(tx)
    )
    .finallyDo(
      () -> stop()
    );
  }
  
}
