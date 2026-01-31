package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public MutDistance position = Inches.mutable(0);
        public MutLinearVelocity velocity = InchesPerSecond.mutable(0);

        public MutVoltage appliedVoltage = Volts.mutable(0);

        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }
    
    void updateInputs(ElevatorIOInputs inputs);
    void runVolts(Voltage volts);
    void runSetpoint(Distance position);
    void stop();
}
