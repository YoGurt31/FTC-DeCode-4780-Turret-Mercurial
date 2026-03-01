package Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import SubSystems.Drive;

@TeleOp(name = "P2P Tuner", group = "Tuning")
@Configurable
public class P2PTuner extends LinearOpMode {

    public static double KP_DIST_STEP = 0.0025;
    public static double KP_ANG_STEP = 0.0010;
    public static double MAX_FWD_STEP = 0.05;
    public static double MAX_TURN_STEP = 0.05;

    public static double DRIVE_TEST_IN = 24.0;
    public static double GOTO_FWD_IN = 12.0;
    public static double GOTO_LEFT_IN = 12.0;
    public static double TURN_TEST_DEG = 90.0;

    private final P2PController p2p = new P2PController();

    private Waypoint[] sequence = null;
    private int seqIndex = 0;

    private final ElapsedTime runTimer = new ElapsedTime();

    private boolean lastA, lastB, lastX, lastY, lastBack, lastStart;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastLB, lastRB;

    private static final class Waypoint {
        final double x;
        final double y;
        final double h;

        Waypoint(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    @Override
    public void runOpMode() {
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(hardwareMap, telemetry);

        Drive.INSTANCE.setPose(0.0, 0.0, 0.0);
        Drive.INSTANCE.updateOdometry();

        if (p2p.kPDist == 0.0) p2p.kPDist = 0.035;
        if (p2p.kPAng == 0.0) p2p.kPAng = 0.025;
        if (p2p.maxFwd <= 0.0) p2p.maxFwd = 0.75;
        if (p2p.maxTurn <= 0.0) p2p.maxTurn = 0.5;

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("P2P Tuner Ready. Press Play.");
        telemetry.addLine("A: turn +90 | B: drive +24 | X: goTo +12F +12L | Y: square | Back: cancel");
        telemetry.addLine("Tune: Dpad (kP), RB/LB (maxFwd), RT/LT (maxTurn)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runTimer.reset();

        while (opModeIsActive()) {
            Drive.INSTANCE.updateOdometry();

            handleLiveTuning();
            handleTestButtons();
            stepSequence();

            publishTelemetry(telemetry);
            telemetry.update();

            idle();
        }

        Drive.INSTANCE.stop();
        p2p.cancel();
    }

    private void handleLiveTuning() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        // Dpad: gains
        if (dpadUp && !lastDpadUp) p2p.kPDist = Math.max(0.0, p2p.kPDist + KP_DIST_STEP);
        if (dpadDown && !lastDpadDown) p2p.kPDist = Math.max(0.0, p2p.kPDist - KP_DIST_STEP);

        if (dpadRight && !lastDpadRight) p2p.kPAng = Math.max(0.0, p2p.kPAng + KP_ANG_STEP);
        if (dpadLeft && !lastDpadLeft) p2p.kPAng = Math.max(0.0, p2p.kPAng - KP_ANG_STEP);

        // Bumpers: max forward
        if (rb && !lastRB) p2p.maxFwd = clamp01(p2p.maxFwd + MAX_FWD_STEP);
        if (lb && !lastLB) p2p.maxFwd = clamp01(p2p.maxFwd - MAX_FWD_STEP);

        // Triggers: max turn (continuous)
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        if (rt > 0.2) p2p.maxTurn = clamp01(p2p.maxTurn + MAX_TURN_STEP * rt);
        if (lt > 0.2) p2p.maxTurn = clamp01(p2p.maxTurn - MAX_TURN_STEP * lt);

        boolean start = gamepad1.start;
        if (start && !lastStart) {
            telemetry.addLine("--- P2P values ---");
            telemetry.addData("kPDist", p2p.kPDist);
            telemetry.addData("kPAng", p2p.kPAng);
            telemetry.addData("kSFwd", p2p.kSFwd);
            telemetry.addData("kSTurn", p2p.kSTurn);
            telemetry.addData("maxFwd", p2p.maxFwd);
            telemetry.addData("maxTurn", p2p.maxTurn);
            telemetry.addData("posTolIn", p2p.posTolIn);
            telemetry.addData("angTolDeg", p2p.angTolDeg);
            telemetry.addData("blendDistIn", p2p.headingBlendDistIn);
        }

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastLB = lb;
        lastRB = rb;
        lastStart = start;
    }

    private void handleTestButtons() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean back = gamepad1.back;

        if (back && !lastBack) {
            sequence = null;
            seqIndex = 0;
            p2p.cancel();
        }

        boolean idle = (sequence == null && !p2p.isBusy());

        if (idle && a && !lastA) {
            startTurnTest(+TURN_TEST_DEG);
        }

        if (idle && b && !lastB) {
            startDriveTest(DRIVE_TEST_IN);
        }

        if (idle && x && !lastX) {
            startGoToTest(GOTO_FWD_IN, GOTO_LEFT_IN);
        }

        if (idle && y && !lastY) {
            startSquareTest(DRIVE_TEST_IN);
        }

        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastBack = back;
    }

    private void startTurnTest(double deltaDeg) {
        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();
        double targetH = Constants.wrapDeg(h + deltaDeg);

        sequence = new Waypoint[]{new Waypoint(x, y, targetH)};
        seqIndex = 0;
        runTimer.reset();

        p2p.beginAbs(sequence[0].x, sequence[0].y, sequence[0].h);
    }

    private void startDriveTest(double forwardIn) {
        Drive.INSTANCE.updateOdometry();
        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();

        double hRad = Math.toRadians(h);
        double tx = x + forwardIn * Math.cos(hRad);
        double ty = y + forwardIn * Math.sin(hRad);

        sequence = new Waypoint[]{new Waypoint(tx, ty, h)};
        seqIndex = 0;
        runTimer.reset();

        p2p.beginAbs(sequence[0].x, sequence[0].y, sequence[0].h);
    }

    private void startGoToTest(double forwardIn, double leftIn) {
        Drive.INSTANCE.updateOdometry();
        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();

        double hRad = Math.toRadians(h);
        double tx = x + forwardIn * Math.cos(hRad) - leftIn * Math.sin(hRad);
        double ty = y + forwardIn * Math.sin(hRad) + leftIn * Math.cos(hRad);

        sequence = new Waypoint[]{new Waypoint(tx, ty, h)};
        seqIndex = 0;
        runTimer.reset();

        p2p.beginAbs(sequence[0].x, sequence[0].y, sequence[0].h);
    }

    private void startSquareTest(double sideIn) {
        Drive.INSTANCE.updateOdometry();
        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h0 = Drive.INSTANCE.getHeading();
        double hRad = Math.toRadians(h0);

        double x1 = x + sideIn * Math.cos(hRad);
        double y1 = y + sideIn * Math.sin(hRad);

        double x2 = x1 - sideIn * Math.sin(hRad);
        double y2 = y1 + sideIn * Math.cos(hRad);

        double x3 = x - sideIn * Math.sin(hRad);
        double y3 = y + sideIn * Math.cos(hRad);

        sequence = new Waypoint[]{
                new Waypoint(x1, y1, h0),
                new Waypoint(x2, y2, h0),
                new Waypoint(x3, y3, h0),
                new Waypoint(x, y, h0)
        };
        seqIndex = 0;
        runTimer.reset();

        p2p.beginAbs(sequence[0].x, sequence[0].y, sequence[0].h);
    }

    private void stepSequence() {
        if (sequence == null || sequence.length == 0) return;

        boolean running = p2p.step();
        if (running) return;

        seqIndex++;
        if (seqIndex >= sequence.length) {
            sequence = null;
            seqIndex = 0;
            return;
        }

        Waypoint wp = sequence[seqIndex];
        p2p.beginAbs(wp.x, wp.y, wp.h);
    }

    private void publishTelemetry(Telemetry t) {
        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();

        double tx = 0.0, ty = 0.0, th = 0.0;
        boolean hasTarget = false;
        if (sequence != null && sequence.length > 0) {
            int idx = Math.min(seqIndex, sequence.length - 1);
            tx = sequence[idx].x;
            ty = sequence[idx].y;
            th = sequence[idx].h;
            hasTarget = true;
        }

        double ex = hasTarget ? (tx - x) : 0.0;
        double ey = hasTarget ? (ty - y) : 0.0;
        double distErr = Math.hypot(ex, ey);
        double angErr = hasTarget ? Constants.wrapDeg(th - h) : 0.0;

        t.addData("Pose (in,deg)", "x=%.2f y=%.2f h=%.1f", x, y, h);
        if (hasTarget) {
            t.addData("Target", "x=%.2f y=%.2f h=%.1f", tx, ty, th);
            t.addData("Error", "dist=%.2f in | ang=%.1f deg", distErr, angErr);
            t.addData("Waypoint", "%d / %d", (seqIndex + 1), sequence.length);
        } else {
            t.addLine("Target: (none)  |  Press A/B/X/Y to start a test");
        }

        t.addData("Busy", p2p.isBusy());
        t.addData("Elapsed", "%.2fs", runTimer.seconds());

        t.addLine("--- Tunables ---");
        t.addData("kPDist", p2p.kPDist);
        t.addData("kPAng", p2p.kPAng);
        t.addData("kSFwd", p2p.kSFwd);
        t.addData("kSTurn", p2p.kSTurn);
        t.addData("maxFwd", p2p.maxFwd);
        t.addData("maxTurn", p2p.maxTurn);
        t.addData("posTolIn", p2p.posTolIn);
        t.addData("angTolDeg", p2p.angTolDeg);
        t.addData("blendDistIn", p2p.headingBlendDistIn);

        t.addLine("Controls: A turn | B drive | X goTo | Y square | Back cancel");
        t.addLine("Tune: Dpad (kP), RB/LB (maxFwd), RT/LT (maxTurn), Start prints values");
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
