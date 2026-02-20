package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO{
    private TalonFX motorLeft = new TalonFX(Constants.ShooterConstants.kMotorLeft);
    private TalonFX motorRight = new TalonFX(Constants.ShooterConstants.kMotorRight);
    private TalonFXSimState simMotorLeft = motorLeft.getSimState();
    private TalonFXSimState simMotorRight = motorRight.getSimState();
    private TalonFXConfiguration configs = new TalonFXConfiguration();

    FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(2), 
        0.002, 
        1),
        DCMotor.getKrakenX60(2)
    );

    AngularVelocity targetVelocity = RPM.of(0);

    
    private double kP = 0.005;
    private double kI = 0;
    private double kD = 0;
    private double kS = 0;
    private double kV = 0.0019891;
    private double kA = 0.00007;
    
    private PIDController pidController = new PIDController(kP, kI, kD, 0.02);
    private SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(kS, kV, kA, 0.02);

    public ShooterIOSim() {
        configs.Slot0.kP = 0.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.0;

        motorRight.setControl(new Follower(Constants.ShooterConstants.kMotorLeft, MotorAlignmentValue.Opposed));

        motorLeft.getConfigurator().apply(configs);
    }


    
    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.magnitude(), -12, 12);
        sim.setInputVoltage(clampedEffort);
    }
    
    @Override
    public void runVelocityRPM(AngularVelocity velocity) {
        Voltage controllerVolts = Volts.of(
            pidController.calculate(sim.getAngularVelocityRPM(), velocity.magnitude()) +
            ffController.calculateWithVelocities(sim.getAngularVelocityRPM(), velocity.magnitude())
        );
        targetVelocity = velocity;
        runVolts(controllerVolts);
    }


    @Override
    public void stop() {
        runVolts(Volts.zero());
    }   
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        sim.update(0.02);
    
        inputs.currentVelocity.mut_replace(RPM.of(sim.getAngularVelocityRPM()));
        inputs.targetVelocity.mut_replace(targetVelocity);
        inputs.currentAcceleration.mut_replace(sim.getAngularAcceleration().div(2*Math.PI).times(60));

        inputs.appliedVoltage.mut_replace(Volts.of(sim.getInputVoltage()));

        inputs.supplyCurrent.mut_replace(Amps.of(sim.getCurrentDrawAmps()));
        inputs.torqueCurrent.mut_replace(Amps.of(sim.getCurrentDrawAmps()));
    }



    @Override
    public void updateGains(double... gains) {
        kP = gains[0];
        kI = gains[1];
        kD = gains[2];
        kS = gains[3];
        kV = gains[4];
        kA = gains[5];

        this.pidController.setPID(kP, kI, kD);
        this.ffController.setKs(kS);
        this.ffController.setKv(kV);
        this.ffController.setKa(kA);
        
    }


}
