package SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import Util.Constants;

public class Vision {
    public static final Vision INSTANCE = new Vision();

    private Vision() {
    }

    private Limelight3A limelight;
    private Telemetry telemetry;

    // Cached result from the latest update()
    private LLResult result;

    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hw.get(Limelight3A.class, Constants.Vision.LIMELIGHT_NAME);

        if (this.telemetry != null) {
            this.telemetry.setMsTransmissionInterval(30);
        }

        limelight.pipelineSwitch(Constants.Vision.DEFAULT_PIPELINE);
        limelight.start();

        result = null;
    }

    public void update() {
        if (limelight == null) return;
        result = limelight.getLatestResult();
    }

    public void stop() {
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

    public boolean hasTag() {
        return getResult() != null;
    }

    public boolean hasFiducial() {
        LLResult r = getResult();
        if (r == null) return false;
        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        return tags != null && !tags.isEmpty();
    }

    public double getTX() {
        LLResult r = getResult();
        return r == null ? 0.0 : r.getTx();
    }

    public double getTA() {
        LLResult r = getResult();
        return r == null ? 0.0 : r.getTa();
    }

    public Pose3D getPose() {
        LLResult r = getResult();
        return r == null ? null : r.getBotpose();
    }

    public String getPipelineName() {
        if (limelight == null) return "Uninitialized";
        if (limelight.getStatus() == null) return "Unknown";

        int idx = limelight.getStatus().getPipelineIndex();
        if (idx == Constants.Vision.DEFAULT_PIPELINE) return "Default";
        if (idx == Constants.Vision.RED_PIPELINE) return "Red";
        if (idx == Constants.Vision.BLUE_PIPELINE) return "Blue";
        return "Unknown";
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
