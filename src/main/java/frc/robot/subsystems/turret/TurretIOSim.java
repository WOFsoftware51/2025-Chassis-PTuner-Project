package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO{
    private final TalonFX motor = new TalonFX(0);
    private final TalonFXSimState motorSim = motor.getSimState();
    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.01, 
            Constants.GearRatios.kTurretRatio
        ),
        DCMotor.getKrakenX60(1)
    );

    private double target = 0;

    private double forwardLimit = Degrees.of(90).in(Rotations)*Constants.GearRatios.kTurretRatio;
    private double reverseLimit = Degrees.of(-90).in(Rotations)*Constants.GearRatios.kTurretRatio;
    private TalonFXConfiguration configs = new TalonFXConfiguration();

    public TurretIOSim() {
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

        configs.MotionMagic.MotionMagicCruiseVelocity = 100;
        configs.MotionMagic.MotionMagicAcceleration = 250;
        
        configs.Slot0.kP = 0.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.0;
        
        // configs.MotorOutput.
        
        motor.getConfigurator().apply(configs);
    }


    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double turretRotations = sim.getAngularPositionRotations();
        double turretRPM = sim.getAngularVelocityRPM();
        
        double rotorRotations = turretRotations * Constants.GearRatios.kTurretRatio;
        double rotorRPS = RPM.of(turretRPM).in(RotationsPerSecond) * Constants.GearRatios.kTurretRatio;
        motorSim.setSupplyVoltage(12.0);
        
        motorSim.setRawRotorPosition(rotorRotations);
        motorSim.setRotorVelocity(rotorRPS);
        
        double positionDegrees = Rotations.of(turretRotations).in(Degrees);
        double velocityDegreesPerSecond = RPM.of(turretRPM).in(DegreesPerSecond);
        
        inputs.position.mut_replace(positionDegrees, Degrees);
        inputs.targetPosition.mut_replace(target, Degrees);

        inputs.velocity.mut_replace(velocityDegreesPerSecond, DegreesPerSecond);
        
        double volts = motorSim.getMotorVoltage();
        inputs.appliedVoltage.mut_replace(volts, Volts);
        sim.setInputVoltage(volts);
        
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        
        sim.update(0.02);
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        motor.setVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Angle degrees) {
        target = (degrees.in(Rotations))*Constants.GearRatios.kTurretRatio;
        MotionMagicVoltage motion = new MotionMagicVoltage(target);
        motor.setControl(motion);
    }

    @Override
    public void stop() {
        motor.setVoltage(0);
    }

    
}
