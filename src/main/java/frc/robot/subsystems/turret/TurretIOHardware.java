package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class TurretIOHardware implements TurretIO {
    private TalonFX motor = new TalonFX(40, "CANivore");
    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private CANcoder canCoder = new CANcoder(1);

    private double forwardLimit = (90.0/360.0)*Constants.GearRatios.kTurretRatio;
    private double reverseLimit = (-90.0/360.0)*Constants.GearRatios.kTurretRatio;


    public TurretIOHardware() {
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
        
        configs.MotionMagic.MotionMagicCruiseVelocity = 80;
        configs.MotionMagic.MotionMagicAcceleration = 1000;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.Slot0.kP = 0.1;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.108;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.2;
        
        // configs.Feedback.SensorToMechanismRatio = Constants.GearRatios.kTurretRatio;
        
        motor.getConfigurator().apply(configs);
        
        // updateEncoder();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double motorRotations = motor.getPosition().getValueAsDouble();
        double motorRPS = motor.getVelocity().getValueAsDouble();
        double motorRPSPS = motor.getAcceleration().getValueAsDouble();

        double turretDegrees = Rotations.of(motorRotations).in(Degrees)/Constants.GearRatios.kTurretRatio;
        double velocityDegreesPerSecond = RotationsPerSecond.of(motorRPS).in(DegreesPerSecond)/Constants.GearRatios.kTurretRatio;
        double velocityDegreesPerSecondPerSecond = RotationsPerSecondPerSecond.of(motorRPSPS).in(DegreesPerSecondPerSecond)/Constants.GearRatios.kTurretRatio;

        inputs.position.mut_replace(turretDegrees, Degrees);
        inputs.velocity.mut_replace(velocityDegreesPerSecond, DegreesPerSecond);
        inputs.acceleration.mut_replace(velocityDegreesPerSecondPerSecond, DegreesPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValueAsDouble(), Amps);

    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        motor.setVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Angle degrees) {
        double target = (degrees.in(Rotations))*Constants.GearRatios.kTurretRatio;
        MotionMagicVoltage motion = new MotionMagicVoltage(target);
        motor.setControl(motion);
    }

    @Override
    public void stop() {
        motor.setVoltage(0);
    }


    public double getCANCoderDegrees() {
        double canCoderDegrees = this.canCoder.getAbsolutePosition().getValueAsDouble()*360; 
        return canCoderDegrees;
    }

  public void updateEncoder(){
    if(canCoder.isConnected()){
      if(getCANCoderDegrees() > 0){
        motor.getConfigurator().setPosition((getCANCoderDegrees()-Constants.GearRatios.kTurretRatio - 360)*Constants.GearRatios.kTurretRatio/360);
      }
      else{
        motor.getConfigurator().setPosition((getCANCoderDegrees()-Constants.GearRatios.kTurretRatio)*Constants.GearRatios.kTurretRatio/360);
      }
    }
    else {
        ErrorHandler.printErrorMessages();
    }
  }

}
