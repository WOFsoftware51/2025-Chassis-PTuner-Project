package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private MutDistance setpoint = Inches.mutable(0);


    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateImputs(inputs);
        Logger.processInputs("Elevator", inputs);

        this.io.runSetpoint(setpoint);

        RobotState.instance.updateElevatorPosition(inputs.position);
    }

    public Command setSetpoint(Distance setpoint) {
        return runOnce(() -> this.setpoint.mut_replace(setpoint));
    }    
}
