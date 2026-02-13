package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.xml.crypto.dsig.Transform;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }
    


    private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();
    private Pose3d pose3d;
    private Pose3d turretLimelightPose3d;

    private Transform3d robotToTurret = new Transform3d();
    private Transform3d turretToLimelight;
    private Transform3d robotToLimelight;

    private double visionLatency;

    private RobotState() {
        turretToLimelight = new Transform3d(
            Constants.TurretConstants.kTurretToLimelightTranslation3d,
            Constants.TurretConstants.kTurretToLimelightRotation3d
        );


    }


    public double getTurretLimelightTX(double tx) {
        return tx;
    }
    public double getTurretLimelightTY(double ty) {
        return ty;
    }

    public ChassisSpeeds setChassisSpeeds(ChassisSpeeds speeds) {
        return robotChassisSpeeds = speeds;
    }
    public ChassisSpeeds getChassisSpeeds() {
        return robotChassisSpeeds;
    }

    public void setTurretLimelightPose3d(Pose3d pose3d) {
        this.turretLimelightPose3d = pose3d;
    }
    public Pose3d getTurretLimelightPose3d() {
        return turretLimelightPose3d;
    }


    public void setRobotToTurret(double turretYawDegrees) {
        robotToTurret = new Transform3d(
            Constants.TurretConstants.kRobotToTurretTranslation3d,
            new Rotation3d(  //TODO
                Degrees.of(0), 
                Degrees.of(0), 
                Degrees.of(turretYawDegrees)
            )
        );
    }
    public Transform3d getRobotToTurret() {
        return robotToTurret;
    }

    public Transform3d getTurretToLimelight(){
        return turretToLimelight;
    }

    
    public Transform3d getRobotToLimelight() {
        robotToLimelight = getRobotToTurret().plus(getTurretToLimelight());
        return robotToLimelight;
    }


    public void setVisionLatency(double latency) {
        this.visionLatency = latency;
    }
    public double getVisionLatency() {
        return this.visionLatency;
    }
}
