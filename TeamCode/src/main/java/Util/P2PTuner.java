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

    public static double KS_STEP = 0.01;
    public static double KP_STEP = 0.0025;
    public static double MAX_STEP = 0.05;

    public static double STEP_MIN = 1e-4;
    public static double STEP_MAX = 0.5;

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

    private enum TuneParam {
        KS_FWD,
        KS_TURN,
        KP_DIST,
        KP_ANG,
        MAX_FWD,
        MAX_TURN
    }

    private TuneParam selected = TuneParam.KS_FWD;

    private double ksStep = KS_STEP;
    private double kpStep = KP_STEP;
    private double maxStep = MAX_STEP;

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
        telemetry.addLine("Tests: A turn | B drive | X goTo | Y square");
        telemetry.addLine("Tune: LB/RB Select Param | DPad Up/Down Change Value | DPad Left/Right Change Step");
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

        if (rb && !lastRB) selected = next(selected);
        if (lb && !lastLB) selected = prev(selected);

        if (dpadRight && !lastDpadRight) bumpStep(selected, +1);
        if (dpadLeft && !lastDpadLeft) bumpStep(selected, -1);

        if (dpadUp && !lastDpadUp) applyDelta(selected, +getStep(selected));
        if (dpadDown && !lastDpadDown) applyDelta(selected, -getStep(selected));

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

    private static TuneParam next(TuneParam p) {
        int i = p.ordinal() + 1;
        TuneParam[] vals = TuneParam.values();
        return vals[i % vals.length];
    }

    private static TuneParam prev(TuneParam p) {
        int i = p.ordinal() - 1;
        TuneParam[] vals = TuneParam.values();
        if (i < 0) i = vals.length - 1;
        return vals[i];
    }

    private double getStep(TuneParam p) {
        switch (p) {
            case KS_FWD:
            case KS_TURN:
                return ksStep;
            case KP_DIST:
            case KP_ANG:
                return kpStep;
            case MAX_FWD:
            case MAX_TURN:
                return maxStep;
            default:
                return kpStep;
        }
    }

    private void bumpStep(TuneParam p, int dir) {
        double mult = (dir > 0) ? 2.0 : 0.5;
        switch (p) {
            case KS_FWD:
            case KS_TURN:
                ksStep = clamp(STEP_MIN, STEP_MAX, ksStep * mult);
                break;
            case KP_DIST:
            case KP_ANG:
                kpStep = clamp(STEP_MIN, STEP_MAX, kpStep * mult);
                break;
            case MAX_FWD:
            case MAX_TURN:
                maxStep = clamp(STEP_MIN, STEP_MAX, maxStep * mult);
                break;
        }
    }

    private void applyDelta(TuneParam p, double delta) {
        switch (p) {
            case KS_FWD:
                p2p.kSFwd = Math.max(0.0, p2p.kSFwd + delta);
                break;
            case KS_TURN:
                p2p.kSTurn = Math.max(0.0, p2p.kSTurn + delta);
                break;
            case KP_DIST:
                p2p.kPDist = Math.max(0.0, p2p.kPDist + delta);
                break;
            case KP_ANG:
                p2p.kPAng = Math.max(0.0, p2p.kPAng + delta);
                break;
            case MAX_FWD:
                p2p.maxFwd = clamp01(p2p.maxFwd + delta);
                break;
            case MAX_TURN:
                p2p.maxTurn = clamp01(p2p.maxTurn + delta);
                break;
        }
    }

    private static double clamp(double lo, double hi, double v) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
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

        t.addLine("== Pose ==");
        t.addData("Robot", "x=%.2f in | y=%.2f in | h=%.1f deg", x, y, h);

        t.addLine("== Target ==");
        if (hasTarget) {
            t.addData("Target", "x=%.2f | y=%.2f | h=%.1f", tx, ty, th);
            t.addData("Error", "dist=%.2f in | ang=%.1f deg", distErr, angErr);
            t.addData("Waypoint", "%d / %d", (seqIndex + 1), sequence.length);
        } else {
            t.addLine("Target: none (press A/B/X/Y to start a test)");
        }

        t.addLine("== Status ==");
        t.addData("Busy", p2p.isBusy());
        t.addData("Elapsed", "%.2fs", runTimer.seconds());

        t.addLine("== Tuning ==");
        t.addData("Selected", selected.name());
        t.addData("Step", "ks=%.4f | kp=%.4f | max=%.3f", ksStep, kpStep, maxStep);

        t.addLine("-- Values --");
        t.addData("kSFwd", p2p.kSFwd);
        t.addData("kSTurn", p2p.kSTurn);
        t.addData("kPDist", p2p.kPDist);
        t.addData("kPAng", p2p.kPAng);
        t.addData("maxFwd", p2p.maxFwd);
        t.addData("maxTurn", p2p.maxTurn);

        t.addLine("-- Tolerances --");
        t.addData("posTolIn", p2p.posTolIn);
        t.addData("angTolDeg", p2p.angTolDeg);
        t.addData("blendDistIn", p2p.headingBlendDistIn);

        t.addLine("== Controls ==");
        t.addLine("Tests: A turn | B drive | X goTo | Y square | Back cancel");
        t.addLine("Tune: LB/RB select param | Dpad Up/Down change value | Dpad Left/Right change step | Start prints values");
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
