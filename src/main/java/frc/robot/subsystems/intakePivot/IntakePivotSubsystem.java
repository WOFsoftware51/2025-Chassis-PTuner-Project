// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
  IntakePivotIO io;
  IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  
  public IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;
  }

  
  public Command runVoltsJoystick(DoubleSupplier volts) {
    return run(() -> 
      io.runVolts(Volts.of(volts.getAsDouble()*12))
    )
    .finallyDo(() ->
      io.stop()
    );
  }


  public Command runVolts(double volts) {
    return run(() ->
      io.runVolts(Volts.of(volts))
    )
    .finallyDo(() ->
      io.stop()
    );
  }

  
  public Command runSetpoint(double degrees) {
    return run(() ->
      io.runSetpoint(Degrees.of(degrees))
    )
    .finallyDo(() -> 
      io.stop()
    );
  } 




  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
  }
}
