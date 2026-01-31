package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Limelight extends SubsystemBase {
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-turret");

    // public double tx = 0.0;
    // public double ty = 0.0;

    // public double target = 0.0;
    // public double error = 0.0;


    @Override
    public void periodic() {
        // tx = table.getEntry("tx").getDouble(tx);
        // ty = table.getEntry("ty").getDouble(ty);

        // error = target + tx;

        // RobotState.instance.getTurretLimelightTX(tx);
        // RobotState.instance.getTurretLimelightTY(tx);
            

        // Logger.recordOutput("Limelight/error", error);
        // Logger.recordOutput("Limelight/tx", tx);
        // Logger.recordOutput("Limelight/ty", ty);
    }
}
