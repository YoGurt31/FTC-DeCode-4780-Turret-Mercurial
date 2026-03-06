package org.firstinspires.ftc.teamcode.Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Drive;

@SuppressWarnings("unused")
@Configurable
public final class P2P_Tuner {

    public static double driveDistanceIn = 24.0;

    public static boolean resetPoseOnStart = false;
    public static double resetXIn = 0.0;
    public static double resetYIn = 0.0;
    public static double resetHeadingDeg = 0.0;

    private enum Mode {
        IDLE,
        DRIVE_DISTANCE,
        TURN_TO
    }

    private static final class State {
        Mode mode = Mode.IDLE;

        double startX;
        double startY;
        double startH;
        double targetDistance;

        double targetHeading;

        long startTimeMs;
        long settleStartMs = -1;

        double lastTraveled;
        double lastRemaining;
        double lastHeadingErr;
    }

    public static final Mercurial.RegisterableProgram p2pTuner = Mercurial.teleop(ctx -> {
        State s = new State();

        ctx.schedule(loop(exec(() -> {
            Drive.INSTANCE.updateOdometry();

            switch (s.mode) {
                case DRIVE_DISTANCE:
                    tickDriveDistance(s);
                    break;
                case TURN_TO:
                    tickTurnTo(s);
                    break;
                case IDLE:
                default:
                    double fwd = -ctx.gamepad1().left_stick_y;
                    double turn = ctx.gamepad1().right_stick_x;

                    if (Math.abs(fwd) < 0.05) fwd = 0.0;
                    if (Math.abs(turn) < 0.05) turn = 0.0;

                    Drive.INSTANCE.drive(Range.clip(fwd, -1.0, 1.0), Range.clip(turn, -1.0, 1.0));
                    break;
            }

            Telemetry t = ctx.telemetry();
            t.addData("Mode", s.mode);

            t.addLine("--- Controls ---");
            t.addLine("DPad Up: +24in   DPad Down: -24in");
            t.addLine("A:0deg  B:90deg  X:-90deg  Y:180deg  Back:Stop");

            t.addLine("--- Odometry ---");
            t.addData("X (in)", Drive.INSTANCE.getX());
            t.addData("Y (in)", Drive.INSTANCE.getY());
            t.addData("H (deg)", Drive.INSTANCE.getHeading());

            t.addLine("--- Live Errors ---");
            t.addData("traveled (in)", s.lastTraveled);
            t.addData("remaining (in)", s.lastRemaining);
            t.addData("headingErr (deg)", s.lastHeadingErr);

            t.addLine("--- PID2Point Tunables ---");
            t.addData("kPDist", PID2Point.kPDist);
            t.addData("kPAng", PID2Point.kPAng);
            t.addData("kSFwd", PID2Point.kSFwd);
            t.addData("kSTurn", PID2Point.kSTurn);
            t.addData("maxFwd", PID2Point.maxFwd);
            t.addData("maxTurn", PID2Point.maxTurn);
            t.addData("distTolIn", PID2Point.distTolIn);
            t.addData("angTolDeg", PID2Point.angTolDeg);
            t.addData("turnSign", PID2Point.turnSign);
            t.addData("timeoutSec", PID2Point.timeoutSec);

            t.update();
        })));

        // --- Buttons ---

        // Stop current action
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().back), exec(() -> stop(s)));

        // DPad drive distance
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_up), exec(() -> startDriveDistance(s, +Math.abs(driveDistanceIn))));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_down), exec(() -> startDriveDistance(s, -Math.abs(driveDistanceIn))));

        // Face buttons turnTo
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().a), exec(() -> startTurnTo(s, 0.0)));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().b), exec(() -> startTurnTo(s, 90.0)));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().x), exec(() -> startTurnTo(s, -90.0)));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().y), exec(() -> startTurnTo(s, 180.0)));

        ctx.dropToScheduler();
    });

    private static void stop(State s) {
        s.mode = Mode.IDLE;
        s.settleStartMs = -1;
        s.lastTraveled = 0.0;
        s.lastRemaining = 0.0;
        s.lastHeadingErr = 0.0;
        Drive.INSTANCE.stop();
    }

    private static void maybeResetPose() {
        if (!resetPoseOnStart) return;
        Drive.INSTANCE.setPose(resetXIn, resetYIn, resetHeadingDeg);
        Drive.INSTANCE.updateOdometry();
    }

    private static void startDriveDistance(State s, double distanceIn) {
        maybeResetPose();

        Drive.INSTANCE.updateOdometry();
        s.startX = Drive.INSTANCE.getX();
        s.startY = Drive.INSTANCE.getY();
        s.startH = Drive.INSTANCE.getHeading();
        s.targetDistance = distanceIn;

        s.startTimeMs = System.currentTimeMillis();
        s.settleStartMs = -1;
        s.mode = Mode.DRIVE_DISTANCE;
    }

    private static void startTurnTo(State s, double headingDeg) {
        maybeResetPose();

        Drive.INSTANCE.updateOdometry();
        s.targetHeading = Constants.wrapDeg(headingDeg);

        s.startTimeMs = System.currentTimeMillis();
        s.settleStartMs = -1;
        s.mode = Mode.TURN_TO;
    }

    // Returns true when finished.
    private static boolean tickDriveDistance(State s) {
        long now = System.currentTimeMillis();

        if (PID2Point.timeoutSec > 0.0) {
            double elapsed = (now - s.startTimeMs) / 1000.0;
            if (elapsed >= PID2Point.timeoutSec) {
                stop(s);
                return true;
            }
        }

        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();

        // project traveled distance along the starting heading
        double hRad = Math.toRadians(s.startH);
        double dx = x - s.startX;
        double dy = y - s.startY;
        double traveled = dx * Math.cos(hRad) + dy * Math.sin(hRad);

        double remaining = s.targetDistance - traveled;
        double headingErrDeg = Constants.wrapDeg(s.startH - h);

        s.lastTraveled = traveled;
        s.lastRemaining = remaining;
        s.lastHeadingErr = headingErrDeg;

        boolean atDist = Math.abs(remaining) <= PID2Point.distTolIn;
        boolean atAng = Math.abs(headingErrDeg) <= PID2Point.angTolDeg;

        if (atDist && atAng) {
            if (s.settleStartMs < 0) s.settleStartMs = now;
            Drive.INSTANCE.drive(0.0, 0.0);
            if (now - s.settleStartMs >= new PID2Point().settleMs) {
                stop(s);
                return true;
            }
            return false;
        } else {
            s.settleStartMs = -1;
        }

        double fwd = Range.clip(remaining * PID2Point.kPDist, -PID2Point.maxFwd, PID2Point.maxFwd);
        if (PID2Point.kSFwd > 0.0 && Math.abs(fwd) > 1e-6) {
            if (Math.abs(fwd) < PID2Point.kSFwd) fwd = PID2Point.kSFwd * Math.signum(fwd);
        }

        double turn = Range.clip((headingErrDeg * PID2Point.kPAng) * PID2Point.turnSign, -PID2Point.maxTurn, PID2Point.maxTurn);
        if (PID2Point.kSTurn > 0.0 && Math.abs(headingErrDeg) > PID2Point.angTolDeg) {
            if (Math.abs(turn) < PID2Point.kSTurn) turn = PID2Point.kSTurn * Math.signum(turn);
        }

        Drive.INSTANCE.drive(fwd, turn);
        return false;
    }

    // Returns true when finished.
    private static boolean tickTurnTo(State s) {
        long now = System.currentTimeMillis();

        if (PID2Point.timeoutSec > 0.0) {
            double elapsed = (now - s.startTimeMs) / 1000.0;
            if (elapsed >= PID2Point.timeoutSec) {
                stop(s);
                return true;
            }
        }

        double h = Drive.INSTANCE.getHeading();
        double headingErrDeg = Constants.wrapDeg(s.targetHeading - h);

        s.lastTraveled = 0.0;
        s.lastRemaining = 0.0;
        s.lastHeadingErr = headingErrDeg;

        boolean atAng = Math.abs(headingErrDeg) <= PID2Point.angTolDeg;
        if (atAng) {
            if (s.settleStartMs < 0) s.settleStartMs = now;
            Drive.INSTANCE.drive(0.0, 0.0);
            if (now - s.settleStartMs >= new PID2Point().settleMs) {
                stop(s);
                return true;
            }
            return false;
        } else {
            s.settleStartMs = -1;
        }

        double turn = Range.clip((headingErrDeg * PID2Point.kPAng) * PID2Point.turnSign, -PID2Point.maxTurn, PID2Point.maxTurn);
        if (PID2Point.kSTurn > 0.0 && Math.abs(headingErrDeg) > PID2Point.angTolDeg) {
            if (Math.abs(turn) < PID2Point.kSTurn) turn = PID2Point.kSTurn * Math.signum(turn);
        }

        Drive.INSTANCE.drive(0.0, turn);
        return false;
    }
}
