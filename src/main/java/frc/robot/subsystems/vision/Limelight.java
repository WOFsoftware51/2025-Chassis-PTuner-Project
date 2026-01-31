package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Limelight extends SubsystemBase {
    public static Limelight instance =  new Limelight();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-turret");

    public double tx = 0.0;
    public double ty = 0.0;


    @Override
    public void periodic() {
        tx = table.getEntry("tx").getDouble(tx);
        ty = table.getEntry("ty").getDouble(ty);

        RobotState.instance.getTurretLimelightTX(tx);
        RobotState.instance.getTurretLimelightTY(tx);
            
    }
}
