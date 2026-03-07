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
        final PID2Point.ActionState action = new PID2Point.ActionState();
    }

    public static final Mercurial.RegisterableProgram p2pTuner = Mercurial.teleop(ctx -> {
        State s = new State();

        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

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
            t.addData("traveled (in)", s.action.lastTraveled);
            t.addData("remaining (in)", s.action.lastRemaining);
            t.addData("headingErr (deg)", s.action.lastHeadingErr);

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
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().back), exec(() -> stop(s)));

        // DPad driveDistance
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_up), exec(() -> startDriveDistance(s, +Math.abs(driveDistanceIn))));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_down), exec(() -> startDriveDistance(s, -Math.abs(driveDistanceIn))));

        // Face Buttons turnTo
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().a), exec(() -> startTurnTo(s, 0.0)));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().b), exec(() -> startTurnTo(s, 90.0)));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().x), exec(() -> startTurnTo(s, -45.0)));
        ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().y), exec(() -> startTurnTo(s, 180.0)));

        ctx.dropToScheduler();
    });

    private static void stop(State s) {
        s.mode = Mode.IDLE;
        s.action.settleStartMs = -1;
        s.action.lastTraveled = 0.0;
        s.action.lastRemaining = 0.0;
        s.action.lastHeadingErr = 0.0;
        s.action.done = true;
        Drive.INSTANCE.stop();
    }

    private static void maybeResetPose() {
        if (!resetPoseOnStart) return;
        Drive.INSTANCE.setPose(resetXIn, resetYIn, resetHeadingDeg);
        Drive.INSTANCE.updateOdometry();
    }

    private static void startDriveDistance(State s, double distanceIn) {
        maybeResetPose();
        PID2Point.beginDriveDistance(s.action, distanceIn);
        s.mode = Mode.DRIVE_DISTANCE;
    }

    private static void startTurnTo(State s, double headingDeg) {
        maybeResetPose();
        PID2Point.beginTurnTo(s.action, headingDeg);
        s.mode = Mode.TURN_TO;
    }

    private static boolean tickDriveDistance(State s) {
        boolean done = PID2Point.tickDriveDistance(s.action);
        if (done) stop(s);
        return done;
    }

    private static boolean tickTurnTo(State s) {
        boolean done = PID2Point.tickTurnTo(s.action);
        if (done) stop(s);
        return done;
    }
}
