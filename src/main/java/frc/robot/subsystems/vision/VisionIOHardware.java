package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.LimelightHelpers.IMUData;

public class VisionIOHardware implements VisionIO {
    public String limelight;

    public VisionIOHardware(String limelightName) {
        this.limelight = limelightName;
    }

    Pose2d test = new Pose2d();

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(limelight);
        inputs.ty = LimelightHelpers.getTY(limelight);
        inputs.tv = LimelightHelpers.getTV(limelight);
        inputs.ta = LimelightHelpers.getTA(limelight);

        inputs.tl = LimelightHelpers.getLatency_Pipeline(limelight);
        inputs.cl = LimelightHelpers.getLatency_Capture(limelight);

        if (inputs.tv) {
            // Pose2d rawPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight).pose;
            Pose2d rawPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
            Pose2d rawMegaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight).pose;
            
            if (Double.isFinite(rawPose.getX()) && Double.isFinite(rawPose.getY()) && Double.isFinite(rawPose.getRotation().getRadians())) {
                inputs.botpose_x = rawPose.getX();
                inputs.botpose_y = rawPose.getY();
                inputs.botpose_rot = rawPose.getRotation().getRadians();

                inputs.MegaTag2_x = rawMegaTag2Pose.getX();
                inputs.MegaTag2_y = rawMegaTag2Pose.getY();
                inputs.MegaTag2_rot = rawMegaTag2Pose.getRotation().getRadians();
            } 
        } 
        
        IMUData imu = LimelightHelpers.getIMUData(limelight);
        inputs.yaw = imu.Yaw;
        inputs.roll = imu.Roll;
        inputs.pitch = imu.Pitch;

        inputs.currentPipeline = LimelightHelpers.getCurrentPipelineIndex(limelight);
    }

    @Override
    public void resetYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetYaw'");
    }

    
}
