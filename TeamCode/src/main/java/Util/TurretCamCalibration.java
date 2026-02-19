package Util;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.concurrent.TimeUnit;
import java.util.Locale;

@TeleOp(name = "Utility: TurretCam Frame Capture", group = "Utility")
public class TurretCamCalibration extends LinearOpMode {
    // TurretCam Calibration
    final String WEBCAM_NAME = "TurretCam";
    final int RESOLUTION_WIDTH = 1280;
    final int RESOLUTION_HEIGHT = 720;

    // Internal State
    boolean lastX;
    int frameCount;
    long capReqTime;

    boolean thisExpUp, thisExpDn, thisGainUp, thisGainDn;
    boolean lastExpUp, lastExpDn, lastGainUp, lastGainDn;

    int myExposureMs;
    int minExposureMs;
    int maxExposureMs;
    int myGain;
    int minGain;
    int maxGain;

    @Override
    public void runOpMode() {
        VisionPortal portal;

        portal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME)).setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).build();

        getCameraSetting(portal);
        myExposureMs = Range.clip(5, minExposureMs, maxExposureMs);
        myGain = maxGain;
        setManualExposure(portal, myExposureMs, myGain);

        telemetry.setMsTransmissionInterval(30);
        telemetry.addLine("TurretCam Calibration Capture Ready");
        telemetry.addData("Webcam", WEBCAM_NAME);
        telemetry.addData("Requested", "%dx%d @ MJPEG (target 30 FPS)", RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
        telemetry.addLine("Press START, then press X to capture a raw frame");
        telemetry.addLine("LB / LT: Exposure + / -   |   RB / RT: Gain + / -");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            boolean x = gamepad1.x;

            // Exposure/Gain Button States
            thisExpUp = gamepad1.left_bumper;
            thisExpDn = gamepad1.left_trigger > 0.25;
            thisGainUp = gamepad1.right_bumper;
            thisGainDn = gamepad1.right_trigger > 0.25;

            if (thisExpUp && !lastExpUp) {
                myExposureMs = Range.clip(myExposureMs + 1, minExposureMs, maxExposureMs);
                setManualExposure(portal, myExposureMs, myGain);
            } else if (thisExpDn && !lastExpDn) {
                myExposureMs = Range.clip(myExposureMs - 1, minExposureMs, maxExposureMs);
                setManualExposure(portal, myExposureMs, myGain);
            }

            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 1, minGain, maxGain);
                setManualExposure(portal, myExposureMs, myGain);
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 1, minGain, maxGain);
                setManualExposure(portal, myExposureMs, myGain);
            }

            if (x && !lastX) {
                portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
                capReqTime = System.currentTimeMillis();
            }

            lastX = x;
            lastExpUp = thisExpUp;
            lastExpDn = thisExpDn;
            lastGainUp = thisGainUp;
            lastGainDn = thisGainDn;

            telemetry.addLine("######## TurretCam Frame Capture ########");
            telemetry.addData("Webcam", WEBCAM_NAME);
            telemetry.addData("Resolution", String.format(Locale.US, "%dx%d", RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
            telemetry.addData("StreamFormat", "MJPEG");
            telemetry.addData("Press X", "Capture raw frame");
            telemetry.addData("Exposure (ms)", "%d  (%d - %d)", myExposureMs, minExposureMs, maxExposureMs);
            telemetry.addData("Gain", "%d  (%d - %d)", myGain, minGain, maxGain);
            telemetry.addData("Camera Status", portal.getCameraState());

            if (capReqTime != 0) {
                telemetry.addLine("\nCaptured Frame!");
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }

    private boolean setManualExposure(VisionPortal portal, int exposureMs, int gain) {
        if (portal == null) return false;

        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for STREAMING...");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (isStopRequested()) return false;

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);
        }

        GainControl gainControl = portal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(gain);
            sleep(20);
        }

        return true;
    }

    private void getCameraSetting(VisionPortal portal) {
        if (portal == null) return;

        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for STREAMING...");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            minExposureMs = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposureMs = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        } else {
            minExposureMs = 1;
            maxExposureMs = 100;
        }

        GainControl gainControl = portal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        } else {
            minGain = 0;
            maxGain = 255;
        }
    }
}
