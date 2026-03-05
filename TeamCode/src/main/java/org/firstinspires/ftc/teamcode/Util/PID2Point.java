package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;

@Configurable
public class PID2Point {

    // TODO: TUNE ALL OF THESE
    public static double kPDist = 0.03;
    public static double kPAng = 0.0085;

    public static double kSFwd = 0.15;
    public static double kSTurn = 0.15;

    public static double maxFwd = 1.00;
    public static double maxTurn = 0.75;

    public static double distTolIn = 0.50;
    public static double angTolDeg = 1.0;

    public static double turnSign = -1.0;

    public static double timeoutSec = 0.0;

    public long settleMs = 500;

    public void DriveDistance(LinearOpMode opMode, double inches) {
        if (opMode == null) return;

        Drive.INSTANCE.updateOdometry();

        final double startX = Drive.INSTANCE.getX();
        final double startY = Drive.INSTANCE.getY();
        final double startH = Drive.INSTANCE.getHeading();

        final long startTime = System.currentTimeMillis();
        long settleStart = -1;

        while (opMode.opModeIsActive()) {
            if (timeoutSec > 0.0) {
                double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;
                if (elapsed >= timeoutSec) break;
            }

            Drive.INSTANCE.updateOdometry();

            double x = Drive.INSTANCE.getX();
            double y = Drive.INSTANCE.getY();
            double h = Drive.INSTANCE.getHeading();

            double hRad = Math.toRadians(startH);
            double dx = x - startX;
            double dy = y - startY;
            double traveled = dx * Math.cos(hRad) + dy * Math.sin(hRad);

            double remaining = inches - traveled;

            double headingErrDeg = Constants.wrapDeg(startH - h);

            boolean atDist = Math.abs(remaining) <= distTolIn;
            boolean atAng = Math.abs(headingErrDeg) <= angTolDeg;

            if (atDist && atAng) {
                if (settleStart < 0) settleStart = System.currentTimeMillis();
                Drive.INSTANCE.drive(0.0, 0.0);
                if (System.currentTimeMillis() - settleStart >= settleMs) break;
                opMode.idle();
                continue;
            } else {
                settleStart = -1;
            }

            double fwd = Range.clip(remaining * kPDist, -maxFwd, maxFwd);

            if (kSFwd > 0.0 && Math.abs(fwd) > 1e-6) {
                if (Math.abs(fwd) < kSFwd) fwd = kSFwd * Math.signum(fwd);
            }

            double turn = Range.clip((headingErrDeg * kPAng) * turnSign, -maxTurn, maxTurn);

            if (kSTurn > 0.0 && Math.abs(headingErrDeg) > angTolDeg) {
                if (Math.abs(turn) < kSTurn) turn = kSTurn * Math.signum(turn);
            }

            Drive.INSTANCE.drive(fwd, turn);
            opMode.idle();
        }

        Drive.INSTANCE.stop();
    }

    public void TurnTo(LinearOpMode opMode, double deg) {
        if (opMode == null) return;

        double targetHeadingDeg = Constants.wrapDeg(deg);

        final long startTime = System.currentTimeMillis();
        long settleStart = -1;

        while (opMode.opModeIsActive()) {
            if (timeoutSec > 0.0) {
                double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;
                if (elapsed >= timeoutSec) break;
            }

            Drive.INSTANCE.updateOdometry();

            double h = Drive.INSTANCE.getHeading();
            double headingErrDeg = Constants.wrapDeg(targetHeadingDeg - h);

            boolean atAng = Math.abs(headingErrDeg) <= angTolDeg;

            if (atAng) {
                if (settleStart < 0) settleStart = System.currentTimeMillis();
                Drive.INSTANCE.drive(0.0, 0.0);
                if (System.currentTimeMillis() - settleStart >= settleMs) break;
                opMode.idle();
                continue;
            } else {
                settleStart = -1;
            }

            double turn = Range.clip((headingErrDeg * kPAng) * turnSign, -maxTurn, maxTurn);

            if (kSTurn > 0.0 && Math.abs(headingErrDeg) > angTolDeg) {
                if (Math.abs(turn) < kSTurn) turn = kSTurn * Math.signum(turn);
            }

            Drive.INSTANCE.drive(0.0, turn);
            opMode.idle();
        }

        Drive.INSTANCE.stop();
    }
}
