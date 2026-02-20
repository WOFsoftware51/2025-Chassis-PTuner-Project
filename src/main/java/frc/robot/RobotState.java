package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }
    
    public Pose2d hubTargetBlue = new Pose2d(4.620, 4.040, new Rotation2d());
    public Pose2d hubTargetRed = new Pose2d(11.915, 4.040, new Rotation2d());

    private Pose2d pose2d = new Pose2d();
    private Angle turretYaw = Degrees.of(0);

    private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();
    private Pose2d turretLimelightPose2d = new Pose2d();
    private Pose2d turretLimelightMegaTag2 = new Pose2d();

    private Translation3d robotToTurreTranslation3d = 
        new Translation3d(
            Inches.of(0), 
            Inches.of(0), 
            Inches.of(8)
        );  
    
    private Transform3d robotToTurret = new Transform3d(
        robotToTurreTranslation3d,
        new Rotation3d(
            0,
            0,
            0
        )
    );

    private Transform3d turretToLimelight = new Transform3d(
        new Translation3d(  //TODO
            Inches.of(0), 
            Inches.of(0), 
            Inches.of(0)
        ),
        new Rotation3d(  //TODO
            Degrees.of(0), 
            Degrees.of(0), 
            Degrees.of(0)
        )
    );
    
    private Transform3d robotToLimelight = new Transform3d();

    private double visionLatency;

    private RobotState() {
    
    }


    public void setPose2d(Pose2d pose) {
        this.pose2d = pose;
    }
    
    public Pose2d getPose2d() {
        return this.pose2d;
    }    

    /**
     * Error from the front of the robot to the hub
     * 
     * @return Angle in Degrees
     */
    public Angle getRobotToHubDegrees() {
        double yError = hubTargetRed.getY() - getPose2d().getY();
        double xError = hubTargetRed.getX() - getPose2d().getX();
        Angle angleRadians = Radians.of(Math.atan2(yError,xError));
        double angleDegrees = angleRadians.in(Degree);
        return Degrees.of(angleDegrees);
    }

    public ChassisSpeeds setChassisSpeeds(ChassisSpeeds speeds) {
        return robotChassisSpeeds = speeds;
    }
    public ChassisSpeeds getChassisSpeeds() {
        return robotChassisSpeeds;
    }

    public void setTurretLimelightPose2d(Pose2d pose2d) {
        this.turretLimelightPose2d = pose2d;
    }
    public Pose2d getTurretLimelightPose2d() {
        return turretLimelightPose2d;
    }

    public void setTurretLimelightMegaTag2(Pose2d pose2d) {
        this.turretLimelightMegaTag2 = pose2d;
    }
    public Pose2d getTurretLimelightMegaTag2() {
        return turretLimelightMegaTag2;
    }

    public void setRobotToTurret(double turretYawDegrees) {
        robotToTurret = new Transform3d(
            robotToTurreTranslation3d,
            new Rotation3d(  //TODO
                Degrees.of(0), 
                Degrees.of(0), 
                Degrees.of(-turretYawDegrees)
            )
        );
    }
    public Transform3d getRobotToTurret() {
        return robotToTurret;
    }

    public Transform3d getTurretToLimelight(){
        return turretToLimelight;
    }

    
    public void setRobotToLimelight() {
        robotToLimelight = getRobotToTurret().plus(getTurretToLimelight());
    }

    public Transform3d getRobotToLimelight() {
        return robotToLimelight;
    }


    public void setVisionLatency(double latency) {
        this.visionLatency = latency;
    }
    public double getVisionLatency() {
        return this.visionLatency;
    }
}
