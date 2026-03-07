package org.firstinspires.ftc.teamcode.Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.continuations.Closure;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;

@Configurable
public class PID2Point {

    public static double kPDist = 0.03;
    public static double kPAng = 0.009;

    public static double kSFwd = 0.15;
    public static double kSTurn = 0.2;

    public static double maxFwd = 1.00;
    public static double maxTurn = 0.8;

    public static double distTolIn = 0.50;
    public static double angTolDeg = 1.0;

    public static double turnSign = -1.0;

    public static double timeoutSec = 0.0;
    public static long settleMs = 500;

    public static final class ActionState {
        public double startX;
        public double startY;
        public double startH;
        public double targetDistance;
        public double targetHeading;
        public long startTimeMs;
        public long settleStartMs = -1;
        public double lastTraveled;
        public double lastRemaining;
        public double lastHeadingErr;
        public boolean done;
    }

    public static void beginDriveDistance(ActionState s, double inches) {
        if (s == null) return;

        Drive.INSTANCE.updateOdometry();
        s.startX = Drive.INSTANCE.getX();
        s.startY = Drive.INSTANCE.getY();
        s.startH = Drive.INSTANCE.getHeading();
        s.targetDistance = inches;
        s.targetHeading = s.startH;
        s.startTimeMs = System.currentTimeMillis();
        s.settleStartMs = -1;
        s.lastTraveled = 0.0;
        s.lastRemaining = inches;
        s.lastHeadingErr = 0.0;
        s.done = false;
    }

    public static void beginTurnTo(ActionState s, double deg) {
        if (s == null) return;

        Drive.INSTANCE.updateOdometry();
        s.startX = Drive.INSTANCE.getX();
        s.startY = Drive.INSTANCE.getY();
        s.startH = Drive.INSTANCE.getHeading();
        s.targetHeading = Constants.wrapDeg(deg);
        s.targetDistance = 0.0;
        s.startTimeMs = System.currentTimeMillis();
        s.settleStartMs = -1;
        s.lastTraveled = 0.0;
        s.lastRemaining = 0.0;
        s.lastHeadingErr = Constants.wrapDeg(s.targetHeading - s.startH);
        s.done = false;
    }

    public static boolean tickDriveDistance(ActionState s) {
        if (s == null) return true;
        if (s.done) return true;

        long now = System.currentTimeMillis();
        if (timeoutSec > 0.0) {
            double elapsed = (now - s.startTimeMs) / 1000.0;
            if (elapsed >= timeoutSec) {
                Drive.INSTANCE.stop();
                s.done = true;
                return true;
            }
        }

        Drive.INSTANCE.updateOdometry();

        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();

        double hRad = Math.toRadians(s.startH);
        double dx = x - s.startX;
        double dy = y - s.startY;
        double traveled = dx * Math.cos(hRad) + dy * Math.sin(hRad);

        double remaining = s.targetDistance - traveled;
        double headingErrDeg = Constants.wrapDeg(s.startH - h);

        s.lastTraveled = traveled;
        s.lastRemaining = remaining;
        s.lastHeadingErr = headingErrDeg;

        boolean atDist = Math.abs(remaining) <= distTolIn;
        boolean atAng = Math.abs(headingErrDeg) <= angTolDeg;

        if (atDist && atAng) {
            if (s.settleStartMs < 0) s.settleStartMs = now;
            Drive.INSTANCE.drive(0.0, 0.0);
            if (now - s.settleStartMs >= settleMs) {
                Drive.INSTANCE.stop();
                s.done = true;
                return true;
            }
            return false;
        }

        s.settleStartMs = -1;

        double fwd = Range.clip(remaining * kPDist, -maxFwd, maxFwd);
        if (kSFwd > 0.0 && Math.abs(fwd) > 1e-6 && Math.abs(fwd) < kSFwd) {
            fwd = kSFwd * Math.signum(fwd);
        }

        double turn = Range.clip((headingErrDeg * kPAng) * turnSign, -maxTurn, maxTurn);
        if (kSTurn > 0.0 && Math.abs(headingErrDeg) > angTolDeg && Math.abs(turn) < kSTurn) {
            turn = kSTurn * Math.signum(turn);
        }

        Drive.INSTANCE.drive(fwd, turn);
        return false;
    }

    public static boolean tickTurnTo(ActionState s) {
        if (s == null) return true;
        if (s.done) return true;

        long now = System.currentTimeMillis();
        if (timeoutSec > 0.0) {
            double elapsed = (now - s.startTimeMs) / 1000.0;
            if (elapsed >= timeoutSec) {
                Drive.INSTANCE.stop();
                s.done = true;
                return true;
            }
        }

        Drive.INSTANCE.updateOdometry();

        double h = Drive.INSTANCE.getHeading();
        double headingErrDeg = Constants.wrapDeg(s.targetHeading - h);

        s.lastTraveled = 0.0;
        s.lastRemaining = 0.0;
        s.lastHeadingErr = headingErrDeg;

        boolean atAng = Math.abs(headingErrDeg) <= angTolDeg;

        if (atAng) {
            if (s.settleStartMs < 0) s.settleStartMs = now;
            Drive.INSTANCE.drive(0.0, 0.0);
            if (now - s.settleStartMs >= settleMs) {
                Drive.INSTANCE.stop();
                s.done = true;
                return true;
            }
            return false;
        }

        s.settleStartMs = -1;

        double turn = Range.clip((headingErrDeg * kPAng) * turnSign, -maxTurn, maxTurn);
        if (kSTurn > 0.0 && Math.abs(headingErrDeg) > angTolDeg && Math.abs(turn) < kSTurn) {
            turn = kSTurn * Math.signum(turn);
        }

        Drive.INSTANCE.drive(0.0, turn);
        return false;
    }

    public static Closure DriveDistance(double inches) {
        ActionState s = new ActionState();
        return sequence(exec(() -> beginDriveDistance(s, inches)), loop(() -> !s.done, exec(() -> tickDriveDistance(s))));
    }

    public static Closure TurnTo(double deg) {
        ActionState s = new ActionState();
        return sequence(exec(() -> beginTurnTo(s, deg)), loop(() -> !s.done, exec(() -> tickTurnTo(s))));
    }
}
