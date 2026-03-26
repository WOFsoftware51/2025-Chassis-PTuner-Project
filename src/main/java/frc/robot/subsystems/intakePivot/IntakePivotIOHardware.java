package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakePivotIOHardware implements IntakePivotIO{
    private TalonFX motor = new TalonFX(Constants.IntakePivotConstants.kMotorID, Constants.kCANIvoreName);
    private CANcoder cancoder = new CANcoder(Constants.IntakePivotConstants.kCANcoderID, Constants.kCANIvoreName);

    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();

    private double forwardLimit = (Constants.IntakePivotConstants.kForwardLimit/360.0)*Constants.IntakePivotConstants.kGearRatio;
    private double reverseLimit = (Constants.IntakePivotConstants.kReverseLimit/360.0)*Constants.IntakePivotConstants.kGearRatio;

    MotionMagicVoltage motion = new MotionMagicVoltage(0);
    double target = 0;


    public IntakePivotIOHardware() {
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.MotionMagic.MotionMagicCruiseVelocity = 30;
        configs.MotionMagic.MotionMagicAcceleration = 30;

        configs.Slot0.kP = 0.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 0.0;
        configs.Slot0.kV = 0.112;
        configs.Slot0.kA = 0.0;

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 40;


        motor.getConfigurator().apply(configs);

        canCoderConfigs.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        cancoder.getConfigurator().apply(canCoderConfigs);

        // cancoder.setPosition(0);

        updateEncoder();
    }


    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.position.mut_replace(motor.getPosition().getValue());

        inputs.velocity.mut_replace(motor.getVelocity().getValue());
        inputs.acceleration.mut_replace(motor.getAcceleration().getValue());

        inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValue());

        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValue());

        inputs.canCoderPosition.mut_replace(cancoder.getPosition().getValue());
    }


    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        motor.setControl(new VoltageOut(clampedEffort).withEnableFOC(true));
    }


    @Override
    public void runSetpoint(Angle degrees) {
        this.target = degrees.in(Degrees);
        this.motion.withPosition(target).withEnableFOC(true);
        motor.setControl(this.motion);
    }


    @Override
    public void resetEncoder() {
        motor.setPosition(0);
    }


    @Override
    public void stop() {
        runVolts(Volts.zero());
    }

    private double getCANCoderRotations() {
        double arm_CANcoder = cancoder.getAbsolutePosition().getValueAsDouble(); 
        return arm_CANcoder;
    }

    private void updateEncoder(){
        if(cancoder.isConnected()){
            motor.getConfigurator().setPosition(((getCANCoderRotations()-Constants.IntakePivotConstants.kCANCoderOffset)/Constants.IntakePivotConstants.kCANCoderGearRatio)*Constants.IntakePivotConstants.kGearRatio);
        }
    }

}
