package Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Tuner - Flywheel PIDF", group = "Tuning")
public class FlyWheelTuner extends LinearOpMode {

    private DcMotorEx fly1, fly2;

    private GoBildaPinpointDriver pinpoint;

    // Variable target RPS tuning
    private final double[] rpsSteps = new double[]{0.1, 0.25, 0.5, 1.0, 2.5};
    private int rpsStepIdx = 1; // 0.25 rps

    private double p = Constants.Flywheel.P;
    private double i = Constants.Flywheel.I;
    private double d = Constants.Flywheel.D;
    private double f = Constants.Flywheel.F;

    private boolean flyOn = false;
    private double targetRps = 52.5;

    private enum Edit {P, F}

    private Edit edit = Edit.P;

    private final double[] steps = new double[]{0.01, 0.1, 0.25, 0.5, 1.0, 5.0};
    private int stepIdx = 3; // 0.5

    private static final class Edge {
        private boolean prev;
        boolean rising(boolean now) {
            boolean r = now && !prev;
            prev = now;
            return r;
        }
    }

    private final Edge aEdge = new Edge();
    private final Edge xEdge = new Edge();
    private final Edge bEdge = new Edge();
    private final Edge dlEdge = new Edge();
    private final Edge drEdge = new Edge();
    private final Edge duEdge = new Edge();
    private final Edge ddEdge = new Edge();
    private final Edge lbEdge = new Edge();
    private final Edge rbEdge = new Edge();
    private final Edge yEdge = new Edge();

    private static double rpsToTicksPerSecond(double rps) {
        return rps * Constants.Flywheel.TICKS_PER_REV;
    }

    @Override
    public void runOpMode() {
        fly1 = hardwareMap.get(DcMotorEx.class, Constants.Flywheel.Flywheel1);
        fly2 = hardwareMap.get(DcMotorEx.class, Constants.Flywheel.Flywheel2);

        // Match your Flywheel subsystem directions
        fly1.setDirection(DcMotorEx.Direction.FORWARD);
        fly2.setDirection(DcMotorEx.Direction.REVERSE);

        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPidf();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PinPoint.PinPoint);
        pinpoint.setEncoderDirections(
                Constants.PinPoint.X_REVERSED
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Constants.PinPoint.Y_REVERSED
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.PinPoint.X_OFFSET_MM, Constants.PinPoint.Y_OFFSET_MM, DistanceUnit.MM);
        // For tuner, reset each run so your logged positions are consistent
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                Constants.PinPoint.START_X_IN,
                Constants.PinPoint.START_Y_IN,
                AngleUnit.DEGREES,
                Constants.PinPoint.START_HEADING_DEG
        ));

        telemetry.addLine("FlyWheelTuner ready");
        telemetry.addLine("A toggle ON/OFF");
        telemetry.addLine("Hold LEFT_STICK_BUTTON + Dpad Up/Down = TargetRPS +/-");
        telemetry.addLine("Dpad Up/Down = step size | Dpad Left=P, Right=F");
        telemetry.addLine("LB/RB adjust selected (P or F) -/+");
        telemetry.addLine("Y cycles RPS adjust step");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry each loop so telemetry is live
            try { pinpoint.update(); } catch (Exception ignored) {}

            // Edge-triggered buttons
            if (aEdge.rising(gamepad1.a)) flyOn = !flyOn;

            if (dlEdge.rising(gamepad1.dpad_left)) edit = Edit.P;
            if (drEdge.rising(gamepad1.dpad_right)) edit = Edit.F;

            // If you HOLD left stick button, Dpad Up/Down adjusts target RPS instead of changing stepIdx
            boolean rpsAdjustMode = gamepad1.left_stick_button;

            if (yEdge.rising(gamepad1.y)) {
                rpsStepIdx = (rpsStepIdx + 1) % rpsSteps.length;
            }
            double rpsStep = rpsSteps[rpsStepIdx];

            if (rpsAdjustMode) {
                if (duEdge.rising(gamepad1.dpad_up)) targetRps += rpsStep;
                if (ddEdge.rising(gamepad1.dpad_down)) targetRps -= rpsStep;
                targetRps = Range.clip(targetRps, 0.0, 100.0);
            } else {
                if (duEdge.rising(gamepad1.dpad_up)) stepIdx = Math.min(steps.length - 1, stepIdx + 1);
                if (ddEdge.rising(gamepad1.dpad_down)) stepIdx = Math.max(0, stepIdx - 1);
            }
            double step = steps[stepIdx];

            boolean changed = false;
            if (lbEdge.rising(gamepad1.left_bumper)) {
                if (edit == Edit.P) p = Math.max(0.0, p - step);
                else f = Math.max(0.0, f - step);
                changed = true;
            }
            if (rbEdge.rising(gamepad1.right_bumper)) {
                if (edit == Edit.P) p = p + step;
                else f = f + step;
                changed = true;
            }

            if (changed) applyPidf();

            if (flyOn) {
                double tps = rpsToTicksPerSecond(targetRps);
                fly1.setVelocity(tps);
                fly2.setVelocity(tps);
            } else {
                fly1.setPower(0);
                fly2.setPower(0);
            }

            double rps1 = fly1.getVelocity() / Constants.Flywheel.TICKS_PER_REV;
            double rps2 = fly2.getVelocity() / Constants.Flywheel.TICKS_PER_REV;
            double avg = (rps1 + rps2) / 2.0;

            Pose2D pose = pinpoint.getPosition();
            double xIn = pose.getX(DistanceUnit.INCH);
            double yIn = pose.getY(DistanceUnit.INCH);
            double hDeg = pose.getHeading(AngleUnit.DEGREES);

            telemetry.addLine("=== PinPoint Pose ===");
            telemetry.addData("X (in)", "%6.2f", xIn);
            telemetry.addData("Y (in)", "%6.2f", yIn);
            telemetry.addData("Heading (deg)", "%6.2f", hDeg);
            telemetry.addLine();

            telemetry.addLine("=== Flywheel PIDF ===");
            telemetry.addData("State", flyOn ? "ON" : "OFF");
            telemetry.addData("TargetRPS", "%5.2f", targetRps);
            telemetry.addData("RPS1", "%5.2f", rps1);
            telemetry.addData("RPS2", "%5.2f", rps2);
            telemetry.addData("Avg", "%5.2f", avg);
            telemetry.addData("Editing", edit);
            telemetry.addData("Step", step);
            telemetry.addData("P", "%6.2f", p);
            telemetry.addData("F", "%6.2f", f);
            telemetry.addLine("Copy to Constants.Flywheel:");
            telemetry.addData("P", "%6.2f", p);
            telemetry.addData("F", "%6.2f", f);
            telemetry.addData("I", "%6.2f", i);
            telemetry.addData("D", "%6.2f", d);
            telemetry.update();

            idle();
        }
    }

    private void applyPidf() {
        fly1.setVelocityPIDFCoefficients(p, i, d, f);
        fly2.setVelocityPIDFCoefficients(p, i, d, f);
    }
}