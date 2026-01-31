package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getKrakenX60(2), 
        50.0, 
        Pounds.of(30).in(Kilograms), 
        Inches.of(2).in(Meters), 
        Inches.of(0).in(Meters), 
        Inches.of(60).in(Meters), 
        true, 
        Inches.of(0).in(Meters)
    );

    private final MutVoltage appliedVolts = Volts.mutable(0);
    private final PIDController pidController = new PIDController(2, 0.0, 0.0);
    private final ElevatorFeedforward ffController = new ElevatorFeedforward(0, 0, 0, 0);

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        appliedVolts.mut_replace(clampedEffort, Volts);
        sim.setInputVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Distance position) {
        Distance currentPosition = Meters.of(sim.getPositionMeters());
        Voltage controllerVoltage = Volts.of(
            pidController.calculate(currentPosition.in(Inches), position.in(Inches)) +   
            ffController.calculate(currentPosition.in(Inches), position.in(Inches))
        );

        runVolts(controllerVoltage);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position.mut_replace(sim.getPositionMeters(), Meters);
        inputs.velocity.mut_replace(sim.getVelocityMetersPerSecond(), MetersPerSecond);
        
        inputs.appliedVoltage.mut_replace(appliedVolts);
        
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);

        sim.update(0.02);
    }
}
