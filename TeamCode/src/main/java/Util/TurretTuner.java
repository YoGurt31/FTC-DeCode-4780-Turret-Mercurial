package Util;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TurretTuner
 *
 * QUICK mode:
 *  - TurretCam OFF
 *  - Uses PinPoint localization to aim turret
 *  - Tunes QuickKp + QuickMaxPower
 *
 * PRECISE mode:
 *  - TurretCam ON
 *  - Uses ONLY AprilTag yaw (ignores localization)
 *  - Tunes PreciseKp + PreciseMaxPower
 */

@TeleOp(name = "Tuner - Turret Quick/Precise", group = "Tuning")
public class TurretTuner extends LinearOpMode {

    private DcMotorEx turret;
    private GoBildaPinpointDriver pinpoint;

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;

    private int trackedTagId = Constants.Vision.RED_TAG_ID;

    private double quickKp = Constants.Turret.QuickKp;
    private double quickMax = Constants.Turret.QuickMaxPower;
    private double preciseKp = Constants.Turret.PreciseKp;
    private double preciseMax = Constants.Turret.PreciseMaxPower;

    private enum Mode {QUICK, PRECISE}

    private Mode mode = Mode.QUICK;

    private final double[] steps = new double[]{0.001, 0.0025, 0.005, 0.01, 0.02, 0.05};
    private int stepIdx = 3;

    private boolean prevY, prevA, prevB, prevDU, prevDD, prevDL, prevDR, prevLB, prevRB;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, Constants.Turret.turretRotation);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PinPoint.PinPoint);

        closeCamera();

        telemetry.setMsTransmissionInterval(30);
        telemetry.addLine("TurretTuner ready");
        telemetry.addLine("Y toggle QUICK/PRECISE");
        telemetry.addLine("A track BLUE | B track RED");
        telemetry.addLine("Dpad Up/Down step | LB/RB Kp -/+ | Dpad Left/Right MaxPower -/+");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (y && !prevY) {
                mode = (mode == Mode.QUICK) ? Mode.PRECISE : Mode.QUICK;
                if (mode == Mode.PRECISE) openCamera();
                else closeCamera();
            }

            if (a && !prevA) trackedTagId = Constants.Vision.BLUE_TAG_ID;
            if (b && !prevB) trackedTagId = Constants.Vision.RED_TAG_ID;

            if (du && !prevDU) stepIdx = Math.min(steps.length - 1, stepIdx + 1);
            if (dd && !prevDD) stepIdx = Math.max(0, stepIdx - 1);
            double step = steps[stepIdx];

            if (lb && !prevLB) {
                if (mode == Mode.QUICK) quickKp = Math.max(0.0, quickKp - step);
                else preciseKp = Math.max(0.0, preciseKp - step);
            }
            if (rb && !prevRB) {
                if (mode == Mode.QUICK) quickKp += step;
                else preciseKp += step;
            }

            if (dl && !prevDL) {
                if (mode == Mode.QUICK) quickMax = Math.max(0.0, quickMax - step);
                else preciseMax = Math.max(0.0, preciseMax - step);
            }
            if (dr && !prevDR) {
                if (mode == Mode.QUICK) quickMax = Math.min(1.0, quickMax + step);
                else preciseMax = Math.min(1.0, preciseMax + step);
            }

            prevY = y;
            prevA = a;
            prevB = b;
            prevDU = du;
            prevDD = dd;
            prevDL = dl;
            prevDR = dr;
            prevLB = lb;
            prevRB = rb;

            try {
                pinpoint.update();
            } catch (Exception ignored) {
            }

            double cmd = 0.0;
            boolean hasTag = false;
            double yawDeg = 0.0;

            if (mode == Mode.QUICK) {
                double robotX = pinpoint.getPosition().getX(DistanceUnit.INCH);
                double robotY = pinpoint.getPosition().getY(DistanceUnit.INCH);
                double robotHeadingDeg = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

                double goalHeadingDeg = Constants.Field.computeGoalHeadingDeg(robotX, robotY, trackedTagId);
                double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
                double currentTurretDeg = getTurretDeg();

                double errDeg = wrapDeg(desiredTurretDeg - currentTurretDeg);
                if (Math.abs(errDeg) > Constants.Turret.QuickDeadband) {
                    cmd = Range.clip(errDeg * quickKp, -quickMax, quickMax);
                }

            } else {
                AprilTagDetection det = findTrackedDetection();
                hasTag = det != null;
                if (hasTag) yawDeg = det.ftcPose.yaw;

                if (hasTag && Math.abs(yawDeg) > Constants.Turret.PreciseDeadband) {
                    cmd = Range.clip(yawDeg * preciseKp, -preciseMax, preciseMax);
                }
            }

            turret.setPower(Range.clip(cmd, -1.0, 1.0));

            telemetry.addLine("=== Turret Tuner ===");
            telemetry.addData("Mode", mode);
            telemetry.addData("TrackedTag", trackedTagId);
            telemetry.addData("Step", step);
            telemetry.addData("QuickKp", "%6.4f", quickKp);
            telemetry.addData("QuickMax", "%5.2f", quickMax);
            telemetry.addData("HasTag", hasTag);
            telemetry.addData("Yaw", "%5.2f", yawDeg);
            telemetry.addData("PreciseKp", "%6.4f", preciseKp);
            telemetry.addData("PreciseMax", "%5.2f", preciseMax);
            telemetry.update();

            idle();
        }

        closeCamera();
        turret.setPower(0.0);
    }

    private void openCamera() {
        if (portal != null) return;
        WebcamName cam = hardwareMap.get(WebcamName.class, Constants.Vision.TURRET_CAM_NAME);
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessor(aprilTag)
                .build();
    }

    private void closeCamera() {
        if (portal != null) {
            try {
                portal.close();
            } catch (Exception ignored) {
            }
            portal = null;
        }
        aprilTag = null;
    }

    private AprilTagDetection findTrackedDetection() {
        if (aprilTag == null) return null;
        List<AprilTagDetection> dets = aprilTag.getDetections();
        if (dets == null) return null;
        for (AprilTagDetection d : dets) {
            if (d != null && d.id == trackedTagId) return d;
        }
        return null;
    }

    private double getTurretDeg() {
        return turret.getCurrentPosition() * Constants.Turret.TurretDegPerTick;
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }
}