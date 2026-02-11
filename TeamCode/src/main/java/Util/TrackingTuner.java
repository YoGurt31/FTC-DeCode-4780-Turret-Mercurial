package Util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Tuner - Limelight Tracking", group = "Tuning")
public class TrackingTuner extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;

    // Tunables (copy into Constants.Drive)
    private double rotateGain = Constants.Drive.ROTATE_GAIN;
    private double maxRotate = Constants.Drive.MAX_ROTATE;
    private final double deadbandDeg = Constants.Drive.ARTIFACT_AIM_DEADBAND_DEG;

    // Step size selection
    private final double[] steps = new double[]{0.001, 0.0025, 0.005, 0.01, 0.02, 0.05};
    private int stepIdx = 3; // 0.01

    private boolean trackingEnabled = true;

    // edge tracking
    private boolean prevA, prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight, prevLB, prevRB;

    @Override
    public void runOpMode() {
        // Drive motors
        frontLeft  = hardwareMap.get(DcMotorEx.class, Constants.Drive.frontLeft);
        frontRight = hardwareMap.get(DcMotorEx.class, Constants.Drive.frontRight);
        backLeft   = hardwareMap.get(DcMotorEx.class, Constants.Drive.backLeft);
        backRight  = hardwareMap.get(DcMotorEx.class, Constants.Drive.backRight);

        // Match your Drive subsystem directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, Constants.Vision.LIMELIGHT_NAME);
        limelight.pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
        limelight.start();

        telemetry.setMsTransmissionInterval(30);
        telemetry.addLine("TrackingTuner ready");
        telemetry.addLine("A toggle tracking | Dpad Up/Down step");
        telemetry.addLine("LB/RB ROTATE_GAIN -/+ | Dpad Left/Right MAX_ROTATE -/+");
        telemetry.addLine("Left stick Y = drive | Right stick X = manual turn when not tracking");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (a && !prevA) trackingEnabled = !trackingEnabled;
            if (du && !prevDpadUp) stepIdx = Math.min(steps.length - 1, stepIdx + 1);
            if (dd && !prevDpadDown) stepIdx = Math.max(0, stepIdx - 1);

            double step = steps[stepIdx];

            if (lb && !prevLB) rotateGain = Math.max(0.0, rotateGain - step);
            if (rb && !prevRB) rotateGain = rotateGain + step;

            if (dl && !prevDpadLeft) maxRotate = Math.max(0.0, maxRotate - step);
            if (dr && !prevDpadRight) maxRotate = Math.min(1.0, maxRotate + step);

            prevA = a;
            prevDpadUp = du;
            prevDpadDown = dd;
            prevDpadLeft = dl;
            prevDpadRight = dr;
            prevLB = lb;
            prevRB = rb;

            // Driver forward/back
            double driveCmd = -gamepad1.left_stick_y;
            double speedScale = (gamepad1.left_trigger > 0.25) ? 0.35 : 1.0;
            driveCmd *= speedScale;

            // Default manual turn
            double turnCmd = gamepad1.right_stick_x * speedScale;

            // Limelight results
            LLResult r = limelight.getLatestResult();
            boolean hasArtifact = false;
            double tx = 0.0;
            double ta = 0.0;

            if (r != null && r.isValid()) {
                // This assumes your artifact pipeline yields color results.
                List<LLResultTypes.ColorResult> artifacts = r.getColorResults();
                hasArtifact = artifacts != null && !artifacts.isEmpty();
                tx = r.getTx();
                ta = r.getTa();
            }

            if (trackingEnabled && hasArtifact) {
                if (Math.abs(tx) <= deadbandDeg) {
                    turnCmd = 0.0;
                } else {
                    turnCmd = Range.clip(tx * rotateGain, -maxRotate, maxRotate);
                }
            }

            // Arcade mix
            double left = driveCmd + turnCmd;
            double right = driveCmd - turnCmd;
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right);

            telemetry.addLine("=== Limelight Tracking ===");
            telemetry.addData("Tracking", trackingEnabled ? "ON" : "OFF");
            telemetry.addData("Has Artifact", hasArtifact);
            telemetry.addData("TX", "%5.2f", tx);
            telemetry.addData("TA", "%5.2f", ta);
            telemetry.addData("Step", step);
            telemetry.addData("ROTATE_GAIN", "%6.4f", rotateGain);
            telemetry.addData("MAX_ROTATE", "%5.2f", maxRotate);
            telemetry.addLine("Copy to Constants.Drive:");
            telemetry.addData("ROTATE_GAIN", "%6.4f", rotateGain);
            telemetry.addData("MAX_ROTATE", "%5.2f", maxRotate);
            telemetry.update();

            idle();
        }

        try { limelight.stop(); } catch (Exception ignored) {}
    }
}