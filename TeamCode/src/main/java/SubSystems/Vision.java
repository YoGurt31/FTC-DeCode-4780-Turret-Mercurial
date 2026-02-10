//package SubSystems;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//import Util.Constants;
//
//public class Vision {
//    public static final Vision INSTANCE = new Vision();
//
//    private Vision() {
//    }
//
//    private Limelight3A limelight;
//    private Telemetry telemetry;
//
//    private LLResult result;
//
//    private VisionPortal turretPortal;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection trackedDetection;
//
//    public static final int BLUE = Constants.Vision.BLUE_TAG_ID;
//    public static final int RED = Constants.Vision.RED_TAG_ID;
//    private int trackedTagId = -1;
//
//    public void init(HardwareMap hw, Telemetry telemetry) {
//        this.telemetry = telemetry;
//        this.limelight = hw.get(Limelight3A.class, Constants.Vision.LIMELIGHT_NAME);
//
//        if (this.telemetry != null) {
//            this.telemetry.setMsTransmissionInterval(30);
//        }
//
//        limelight.pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
//        limelight.start();
//
//        WebcamName turretCam = hw.get(WebcamName.class, Constants.Vision.TURRET_CAM_NAME);
//        aprilTag = new AprilTagProcessor.Builder().build();
//        turretPortal = new VisionPortal.Builder()
//                .setCamera(turretCam)
//                .addProcessor(aprilTag)
//                .build();
//
//        result = null;
//        trackedDetection = null;
//    }
//
//    public void update() {
//        if (limelight != null) {
//            result = limelight.getLatestResult();
//        }
//
//        trackedDetection = null;
//        if (aprilTag == null) return;
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//        if (detections == null) return;
//
//        for (AprilTagDetection d : detections) {
//            if (d != null && d.id == trackedTagId) {
//                trackedDetection = d;
//                break;
//            }
//        }
//    }
//
//    public void stop() {
//        if (limelight != null) {
//            try {
//                limelight.stop();
//            } catch (Exception ignored) {
//            }
//        }
//        if (turretPortal != null) {
//            try {
//                turretPortal.close();
//            } catch (Exception ignored) {
//            }
//        }
//    }
//
//    // TODO: FIX THIS SHIT
//    public LLResult getResult() {
//        return (result != null && result.isValid()) ? result : null;
//    }
//
//    public boolean hasTarget() {
//        return getResult() != null;
//    }
//
//    public boolean hasArtifact() {
//        LLResult r = getResult();
//        if (r == null) return false;
//        List<LLResultTypes.ColorResult> tags = r.getColorResults();
//        return tags != null && !tags.isEmpty();
//    }
//
//    public double getTX() {
//        LLResult r = getResult();
//        return r == null ? 0.0 : r.getTx();
//    }
//
//    public double getTA() {
//        LLResult r = getResult();
//        return r == null ? 0.0 : r.getTa();
//    }
//
//    public Pose3D getPose() {
//        LLResult r = getResult();
//        return r == null ? null : r.getBotpose();
//    }
//
//    public String getPipelineName() {
//        if (limelight == null) return "Uninitialized";
//        if (limelight.getStatus() == null) return "Unknown";
//
//        int idx = limelight.getStatus().getPipelineIndex();
//        if (idx == Constants.Vision.DEFAULT_PIPELINE || idx == Constants.Vision.ARTIFACT_PIPELINE) return "Artifacts";
//        return "Unknown";
//    }
//
//    public void setTrackedTag(int tagId) {
//        trackedTagId = tagId;
//    }
//
//    public int getTrackedTag() {
//        return trackedTagId;
//    }
//
//    public boolean hasTrackedTag() {
//        return trackedDetection != null;
//    }
//
//    public double getTrackedYawDeg() {
//        return trackedDetection == null ? 0.0 : trackedDetection.ftcPose.yaw;
//    }
//
//    public double getTrackedRange() {
//        return trackedDetection == null ? 0.0 : trackedDetection.ftcPose.range;
//    }
//
//    public AprilTagDetection getTrackedDetection() {
//        return trackedDetection;
//    }
//
//    public Limelight3A getLimelight() {
//        return limelight;
//    }
//}
