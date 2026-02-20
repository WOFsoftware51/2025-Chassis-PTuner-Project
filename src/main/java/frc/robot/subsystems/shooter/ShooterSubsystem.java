package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/ShooterRPM", 0.0);
    LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
    LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0.0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
    LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", 0.0);

    public boolean gainsChanged = false;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }




    @Override
    public void periodic() {
        io.updateInputs(inputs);


        if(
            kP.hasChanged(kP.hashCode()) ||
            kI.hasChanged(kI.hashCode()) ||
            kD.hasChanged(kD.hashCode()) ||
            kV.hasChanged(kV.hashCode()) ||
            kA.hasChanged(kA.hashCode()) ||
            kS.hasChanged(kS.hashCode())
        ) {
            gainsChanged = true;
        }
        else {
            gainsChanged = false;
        }


        
        Logger.processInputs("Shooter", inputs);
    }

    public Command runRPMCommand() {
        return run(() ->
            io.runVelocityRPM(RPM.of(shooterRPM.get()))
        )
        .finallyDo(() ->
            io.stop()
        );
    }

    public Command updateGainsCommand() {
        return runOnce(() ->
            {
                io.updateGains(
                    kP.get(),
                    kI.get(), 
                    kD.get(),  
                    kS.get(), 
                    kV.get(), 
                    kA.get()
                );
                System.out.println("Shooter Gains Updated!");
            }
        );
    }


}
