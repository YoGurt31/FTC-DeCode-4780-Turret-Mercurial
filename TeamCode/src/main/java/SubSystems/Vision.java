package SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import Util.Constants;

public class Vision {
    public static final Vision INSTANCE = new Vision();

    private Vision() {
    }

    private Telemetry telemetry;

    // Limelight (AprilTag / Localization)
    private Limelight3A limelight;
    private LLResult result;
    private int desiredPipeline = 0;
    private int lastPipeline = -1;

    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        if (this.telemetry != null) {
            this.telemetry.setMsTransmissionInterval(30);
        }

        initLimelight(hw);

        result = null;
    }

    // Limelight (AprilTag / Localization)
    private void initLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, Constants.Vision.LIMELIGHT_NAME);
        limelight.start();
        lastPipeline = -1;
        setPipeline(Constants.Vision.LOCALIZATION_PIPELINE);
    }

    public void update() {
        updateLimelight();
    }

    private void updateLimelight() {
        if (limelight != null) {
            result = limelight.getLatestResult();
        }
    }

    private void stop() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {
            }
        }
    }

    public LLResult getResult() {
        return (result != null && result.isValid()) ? result : null;
    }

    public double getTX() {
        LLResult r = getResult();
        return r == null ? 0.0 : r.getTx();
    }

    public double getTY() {
        LLResult r = getResult();
        return r == null ? 0.0 : r.getTy();
    }

    public double getTA() {
        LLResult r = getResult();
        return r == null ? 0.0 : r.getTa();
    }

    public int getTagId() {
        LLResultTypes.FiducialResult tag = getTag();
        if (tag == null) return -1;
        try { return tag.getFiducialId(); } catch (Exception ignored) { return -1; }
    }


    public LLResultTypes.FiducialResult getTag() {
        LLResult r = getResult();
        if (r == null) return null;

        try { java.util.List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
            if (tags == null || tags.isEmpty()) return null;

            LLResultTypes.FiducialResult best = null;
            double bestArea = -1.0;

            for (LLResultTypes.FiducialResult t : tags) {
                if (t == null) continue;
                if (t.getTargetArea() > bestArea) {
                    bestArea = t.getTargetArea();
                    best = t;
                }
            }

            return best; } catch (Throwable ignored) { return null; }
    }

    public boolean hasTag() {
        return getTag() != null;
    }

    public Pose3D getPose() {
        LLResult r = getResult();
        if (r == null) return null;
        try { return r.getBotpose(); } catch (Throwable ignored) { return null; }
    }

    public boolean hasPose() {
        return getPose() != null;
    }

    public void updateRobotYawDeg(double yawDeg) {
        if (limelight == null) return;
        try { limelight.updateRobotOrientation(yawDeg); } catch (Exception ignored) {}
    }

    public void setPipeline(int pipelineIndex) {
        desiredPipeline = pipelineIndex;
        if (limelight == null) return;
        if (desiredPipeline == lastPipeline) return;
        try {
            limelight.pipelineSwitch(desiredPipeline);
            lastPipeline = desiredPipeline;
        } catch (Exception ignored) { }
    }

    public int getPipelineIndex() {
        if (limelight == null || limelight.getStatus() == null) return -1;
        return limelight.getStatus().getPipelineIndex();
    }

    public String getPipelineName() {
        if (limelight == null) return "Uninitialized";
        if (limelight.getStatus() == null) return "Unknown";

        int idx = limelight.getStatus().getPipelineIndex();
        if (idx == Constants.Vision.DEFAULT_PIPELINE || idx == Constants.Vision.LOCALIZATION_PIPELINE) return "AprilTag";
        return "Unknown";
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
