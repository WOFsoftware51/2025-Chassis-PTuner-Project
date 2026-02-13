package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;


public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public double tx;
        public double ty;
        public boolean tv;
        public double ta;

        public double tl;
        public double cl;

        public Pose3d botpose;
        
        public double yaw;
        public double roll;
        public double pitch;
        
        public double currentPipeline;

    }

    void updateInputs(VisionIOInputs inputs);
    void resetYaw();
}
