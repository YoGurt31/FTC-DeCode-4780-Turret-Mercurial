package SubSystems;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import Util.Constants;

public class Vision {
    public static final Vision INSTANCE = new Vision();

    private Vision() {
    }

    private Telemetry telemetry;

    // Limelight (Artifact Pipeline)
    private Limelight3A limelight;
    private LLResult result;
    private int desiredPipeline = 0;
    private int lastPipeline = -1;

    // TurretCam (AprilTag)
    private VisionPortal turretPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection trackedDetection;
    private int trackedTagId = -1;

    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        if (this.telemetry != null) {
            this.telemetry.setMsTransmissionInterval(30);
        }

        initLimelight(hw);
        initTurretCam(hw);

        result = null;
        trackedDetection = null;
    }

    // Limelight (Artifact Pipeline)
    private void initLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, Constants.Vision.LIMELIGHT_NAME);
        limelight.start();
        lastPipeline = -1;
        setPipeline(Constants.Vision.LOCALIZATION_PIPELINE);
    }

    // TurretCam (AprilTag)
    private void initTurretCam(HardwareMap hw) {
        WebcamName turretCam = hw.get(WebcamName.class, Constants.Vision.TURRET_CAM_NAME);
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(Constants.Vision.INTRINSIC_FX, Constants.Vision.INTRINSIC_FY, Constants.Vision.INTRINSIC_CX, Constants.Vision.INTRINSIC_CY)
                .setCameraPose(new Position(DistanceUnit.INCH, 0, 0, 14, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 15.0, 180.0,0))
                .build();
        turretPortal = new VisionPortal.Builder()
                .setCamera(turretCam)
                .setCameraResolution(new Size(Constants.Vision.RESOLUTION_WIDTH, Constants.Vision.RESOLUTION_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
    }

    public void update() {
        updateLimelight();
        updateTurretCam();
    }

    private void updateLimelight() {
        if (limelight != null) {
            result = limelight.getLatestResult();
        }
    }

    private void updateTurretCam() {
        trackedDetection = null;
        if (aprilTag == null) return;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return;

        for (AprilTagDetection d : detections) {
            if (d != null && d.id == trackedTagId) {
                trackedDetection = d;
                break;
            }
        }
    }

    public void stop() {
        stopLimelight();
        stopTurretCam();
    }

    private void stopLimelight() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {
            }
        }
    }

    private void stopTurretCam() {
        if (turretPortal != null) {
            try {
                turretPortal.close();
            } catch (Exception ignored) {
            }
        }
    }

    // XXX: Limelight Functions
    public LLResult getResult() {
        return (result != null && result.isValid()) ? result : null;
    }

    public boolean hasArtifact() {
        LLResult r = getResult();
        if (r == null) return false;
        List<LLResultTypes.ColorResult> artifacts = r.getColorResults();
        return artifacts != null && !artifacts.isEmpty();
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

    public Pose3D getPose() {
        LLResult r = getResult();
        if (r == null) return null;
        try { return r.getBotpose_MT2(); } catch (Throwable ignored) { return null; }
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
        if (idx == Constants.Vision.DEFAULT_PIPELINE || idx == Constants.Vision.LOCALIZATION_PIPELINE) return "Localization";
        if (idx == Constants.Vision.ARTIFACT_PIPELINE) return "Artifact";
        return "Unknown";
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    // XXX: TurretCam (AprilTag) Functions
    public void setTrackedTag(int tagId) {
        trackedTagId = tagId;
    }

    public int getTrackedTag() {
        return trackedTagId;
    }

    public boolean hasTrackedTag() {
        return trackedDetection != null;
    }

    public double getTrackedCenterX() {
        return trackedDetection == null ? 0.0 : trackedDetection.center.x;
    }

    public double getTrackedCenterError() {
        if (trackedDetection == null) return 0.0;
        return trackedDetection.center.x - (Constants.Vision.RESOLUTION_WIDTH / 2.0);
    }

    public double getTrackedTagCX() {
        if (trackedDetection == null) return 0.0;

        double tagCx = trackedDetection.center.x;
        double imgCx = (Constants.Vision.RESOLUTION_WIDTH / 2.0);
        double errPx = tagCx - imgCx;

        return errPx * Constants.Vision.TURRET_DEG_PER_PIXEL;
    }

    public double getTrackedRange() {
        return trackedDetection == null ? 0.0 : trackedDetection.ftcPose.range;
    }

    public AprilTagDetection getTrackedDetection() {
        return trackedDetection;
    }
}
