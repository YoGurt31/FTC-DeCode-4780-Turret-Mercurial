package Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Tuner - Flywheel PIDF", group = "Tuning")
public class FlyWheelTuner extends LinearOpMode {

    private DcMotorEx fly1, fly2;

    private double p = Constants.Flywheel.P;
    private double i = Constants.Flywheel.I;
    private double d = Constants.Flywheel.D;
    private double f = Constants.Flywheel.F;

    private boolean flyOn = false;
    private double targetRps = Constants.Flywheel.FAR_TARGET_RPS;

    private enum Edit {P, F}

    private Edit edit = Edit.P;

    private final double[] steps = new double[]{0.01, 0.1, 0.25, 0.5, 1.0, 5.0};
    private int stepIdx = 3; // 1.0

    private boolean prevA, prevX, prevB, prevDL, prevDR, prevDU, prevDD, prevLB, prevRB;

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

        telemetry.setMsTransmissionInterval(30);
        telemetry.addLine("FlyWheelTuner ready");
        telemetry.addLine("A toggle ON/OFF | X close RPS | B far RPS");
        telemetry.addLine("Dpad Left=P, Right=F | Dpad Up/Down step");
        telemetry.addLine("LB/RB adjust selected -/+");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;
            boolean dl = gamepad1.dpad_left;
            boolean dr = gamepad1.dpad_right;
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (a && !prevA) flyOn = !flyOn;
            if (x && !prevX) targetRps = Constants.Flywheel.CLOSE_TARGET_RPS;
            if (b && !prevB) targetRps = Constants.Flywheel.FAR_TARGET_RPS;

            if (dl && !prevDL) edit = Edit.P;
            if (dr && !prevDR) edit = Edit.F;

            if (du && !prevDU) stepIdx = Math.min(steps.length - 1, stepIdx + 1);
            if (dd && !prevDD) stepIdx = Math.max(0, stepIdx - 1);
            double step = steps[stepIdx];

            boolean changed = false;
            if (lb && !prevLB) {
                if (edit == Edit.P) p = Math.max(0.0, p - step);
                else f = Math.max(0.0, f - step);
                changed = true;
            }
            if (rb && !prevRB) {
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

            prevA = a;
            prevX = x;
            prevB = b;
            prevDL = dl;
            prevDR = dr;
            prevDU = du;
            prevDD = dd;
            prevLB = lb;
            prevRB = rb;

            idle();
        }
    }

    private void applyPidf() {
        fly1.setVelocityPIDFCoefficients(p, i, d, f);
        fly2.setVelocityPIDFCoefficients(p, i, d, f);
    }
}