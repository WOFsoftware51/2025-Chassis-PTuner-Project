package frc.robot.subsystems.vision;


public class VisionIOHardware implements VisionIO {
    public String limelight;

    public VisionIOHardware(String limelightName) {
        this.limelight = limelightName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(limelight);
        inputs.ty = LimelightHelpers.getTY(limelight);
        inputs.tv = LimelightHelpers.getTV(limelight);
        inputs.ta = LimelightHelpers.getTA(limelight);

        inputs.tl = LimelightHelpers.getLatency_Pipeline(limelight);
        inputs.cl = LimelightHelpers.getLatency_Capture(limelight);

        inputs.botpose = LimelightHelpers.getBotPose3d(limelight);
        
        inputs.yaw = LimelightHelpers.getIMUData(limelight).Yaw;
        inputs.roll = LimelightHelpers.getIMUData(limelight).Roll;
        inputs.pitch = LimelightHelpers.getIMUData(limelight).Pitch;

        inputs.currentPipeline = LimelightHelpers.getCurrentPipelineIndex(limelight);
    }

    @Override
    public void resetYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetYaw'");
    }

    
}
