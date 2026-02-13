package Util;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import SubSystems.Turret;

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
 *
 * Also includes driving (tank) so you can move around while tuning.
 */

@TeleOp(name = "Tuner - Turret Quick/Precise", group = "Tuning")
public class TurretTuner extends LinearOpMode {

    private DcMotorEx turret;

    // Drivetrain motors (for driving during tuning)
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private GoBildaPinpointDriver pinpoint;

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;

    private Constants.Field.Alliance alliance = Constants.Field.Alliance.RED;
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
        // Turret motor
        turret = hardwareMap.get(DcMotorEx.class, Constants.Turret.turretRotation);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0.0);

        // Initialize subsystem wrapper so we can reuse the exact helper methods in Turret.java
        Turret.INSTANCE.init(hardwareMap, telemetry);
        Turret.INSTANCE.zeroTurret();

        // Drivetrain motors (match your Drive.java directions)
        frontLeft  = hardwareMap.get(DcMotorEx.class, Constants.Drive.frontLeft);
        frontRight = hardwareMap.get(DcMotorEx.class, Constants.Drive.frontRight);
        backLeft   = hardwareMap.get(DcMotorEx.class, Constants.Drive.backLeft);
        backRight  = hardwareMap.get(DcMotorEx.class, Constants.Drive.backRight);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PinPoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PinPoint.PinPoint);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(
                Constants.PinPoint.X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Constants.PinPoint.Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.PinPoint.X_OFFSET_MM, Constants.PinPoint.Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                Constants.Field.RED_FAR_X,
                Constants.Field.RED_FAR_Y,
                AngleUnit.DEGREES,
                Constants.Field.RED_FAR_HEADING_DEG
        ));

        closeCamera();

        telemetry.setMsTransmissionInterval(30);
        telemetry.addLine("TurretTuner ready");
        telemetry.addLine("Y toggle QUICK/PRECISE");
        telemetry.addLine("A track BLUE | B track RED");
        telemetry.addLine("Dpad Up/Down step");
        telemetry.addLine("LB/RB adjust Kp -/+");
        telemetry.addLine("Dpad Left/Right adjust MaxPower -/+");
        telemetry.addLine("Drive: LS Y forward/back, RS X turn (LT = slow mode)");
        telemetry.addLine("PinPoint seeded (updates after START)");
        telemetry.update();

        waitForStart();

        Pose2D startPose = new Pose2D(
                DistanceUnit.INCH,
                Constants.Field.RED_FAR_X,
                Constants.Field.RED_FAR_Y,
                AngleUnit.DEGREES,
                Constants.Field.RED_FAR_HEADING_DEG
        );

        pinpoint.setPosition(startPose);

        try { pinpoint.update(); } catch (Exception ignored) {}

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

            // Toggle tuning mode
            if (y && !prevY) {
                mode = (mode == Mode.QUICK) ? Mode.PRECISE : Mode.QUICK;
                if (mode == Mode.PRECISE) openCamera();
                else closeCamera();
            }

            // Choose alliance tag to track
            if (a && !prevA) {
                trackedTagId = Constants.Vision.BLUE_TAG_ID;
                alliance = Constants.Field.Alliance.BLUE;
            }
            if (b && !prevB) {
                trackedTagId = Constants.Vision.RED_TAG_ID;
                alliance = Constants.Field.Alliance.RED;
            }

            // Step size
            if (du && !prevDU) stepIdx = Math.min(steps.length - 1, stepIdx + 1);
            if (dd && !prevDD) stepIdx = Math.max(0, stepIdx - 1);
            double step = steps[stepIdx];

            // Adjust Kp
            if (lb && !prevLB) {
                if (mode == Mode.QUICK) quickKp = Math.max(0.0, quickKp - step);
                else preciseKp = Math.max(0.0, preciseKp - step);
            }
            if (rb && !prevRB) {
                if (mode == Mode.QUICK) quickKp += step;
                else preciseKp += step;
            }

            // Adjust MaxPower
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

            // Update PinPoint
            try {
                pinpoint.update();
            } catch (Exception ignored) {
            }
            Pose2D p = pinpoint.getPosition();

            // --- Turret command using the same helper methods as Turret.java ---
            double cmd = 0.0;
            boolean hasTag = false;
            double yawDeg = 0.0;

            if (mode == Mode.QUICK) {
                double robotX = p.getX(DistanceUnit.INCH);
                double robotY = p.getY(DistanceUnit.INCH);
                double robotHeadingDeg = p.getHeading(AngleUnit.DEGREES);

                double goalHeadingDeg = Constants.Field.computeGoalHeadingDeg(robotX, robotY, alliance);
                double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
                double currentTurretDeg = Turret.INSTANCE.getTurretDeg();

                double errDeg = Turret.INSTANCE.turretErrDeg(desiredTurretDeg, currentTurretDeg);

                if (Math.abs(errDeg) > Constants.Turret.QuickDeadband) {
                    cmd = Range.clip(errDeg * quickKp, -quickMax, quickMax);
                } else {
                    cmd = 0.0;
                }

            } else {
                AprilTagDetection det = findTrackedDetection();
                hasTag = det != null;
                if (hasTag) yawDeg = det.ftcPose.yaw;

                if (hasTag && Math.abs(yawDeg) > Constants.Turret.PreciseDeadband) {
                    cmd = Range.clip(yawDeg * preciseKp, -preciseMax, preciseMax);
                } else {
                    cmd = 0.0;
                }
            }

            cmd = Turret.INSTANCE.applyTurretLimitsToPower(cmd);
            if (Math.abs(cmd) < 0.0001) Turret.INSTANCE.stopTurret();
            else Turret.INSTANCE.setTurretPower(cmd);

            // --- Drive (tank) ---
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            double slow = 1.0 - (0.5 * Range.clip(gamepad1.left_trigger, 0.0, 1.0));

            if (Math.abs(drive) < 0.05) drive = 0.0;
            if (Math.abs(turn) < 0.05) turn = 0.0;

            double left = (drive + turn) * slow;
            double right = (drive - turn) * slow;

            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) { left /= max; right /= max; }

            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right);

            telemetry.addLine("=== Turret Tuner ===");
            telemetry.addData("Mode", mode);
            telemetry.addData("Alliance", alliance);
            telemetry.addData("TrackedTag", trackedTagId);
            telemetry.addData("Step", step);
            telemetry.addData("QuickKp", "%6.4f", quickKp);
            telemetry.addData("QuickMax", "%5.2f", quickMax);
            telemetry.addData("HasTag", hasTag);
            telemetry.addData("Yaw", "%5.2f", yawDeg);
            telemetry.addData("PreciseKp", "%6.4f", preciseKp);
            telemetry.addData("PreciseMax", "%5.2f", preciseMax);

            telemetry.addData("PinPoint X", "%6.2f", p.getX(DistanceUnit.INCH));
            telemetry.addData("PinPoint Y", "%6.2f", p.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%6.2f", p.getHeading(AngleUnit.DEGREES));

            double goalHeading = Constants.Field.computeGoalHeadingDeg(
                    p.getX(DistanceUnit.INCH),
                    p.getY(DistanceUnit.INCH),
                    alliance
            );
            double desiredTurret = wrapDeg(goalHeading - p.getHeading(AngleUnit.DEGREES));

            telemetry.addData("GoalHeading", "%6.2f", goalHeading);
            telemetry.addData("DesiredTurretDeg", "%6.2f", desiredTurret);
            telemetry.addData("TurretDeg", "%6.2f", Turret.INSTANCE.getTurretDeg());

            telemetry.addData("GoalHeading RED",
                    "%6.2f", Constants.Field.computeGoalHeadingDeg(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), Constants.Field.Alliance.RED));
            telemetry.addData("GoalHeading BLUE",
                    "%6.2f", Constants.Field.computeGoalHeadingDeg(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), Constants.Field.Alliance.BLUE));
            telemetry.update();

            idle();
        }

        closeCamera();
        Turret.INSTANCE.stopTurret();
        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }

    private void openCamera() {
        if (portal != null) return;
        WebcamName cam = hardwareMap.get(WebcamName.class, Constants.Vision.TURRET_CAM_NAME);
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1280, 720))
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

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }
}