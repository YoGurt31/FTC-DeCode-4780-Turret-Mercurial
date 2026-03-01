package Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import SubSystems.Drive;

@Configurable
public class P2PTankController {

    // TODO: TUNE ALL OF THESE
    public double kPDist = 0.00;
    public double kPAng = 0.00;

    public double kSFwd = 0.00;
    public double kSTurn = 0.00;

    public double maxFwd = 1.00;
    public double maxTurn = 0.80;

    public double posTolIn = 0.75;
    public double angTolDeg = 2.0;

    public double headingBlendDistIn = 8.0;

    public boolean useFacingScale = true;

    public double turnSign = 1.0;

    public double timeoutSec = 0.0;
    public long settleMs = 150;

    private boolean busy;
    private long startTimeMs;
    private long settleStartMs = -1;

    private double targetXAbs;
    private double targetYAbs;
    private double targetHAbs;

    public void goToPose(LinearOpMode opMode, double targetXIn, double targetYIn, double targetHeadingDeg) {
        if (opMode == null) return;

        final long startTime = System.currentTimeMillis();
        long settleStart = -1;

        targetHeadingDeg = Constants.wrapDeg(targetHeadingDeg);

        while (opMode.opModeIsActive()) {
            if (timeoutSec > 0.0) {
                double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;
                if (elapsed >= timeoutSec) break;
            }

            Drive.INSTANCE.updateOdometry();

            double x = Drive.INSTANCE.getX();
            double y = Drive.INSTANCE.getY();
            double h = Drive.INSTANCE.getHeading();

            double ex = targetXIn - x;
            double ey = targetYIn - y;
            double dist = Math.hypot(ex, ey);

            double angleToPointDeg = Math.toDegrees(Math.atan2(ey, ex));

            double desiredHeadingDeg = (dist > headingBlendDistIn) ? angleToPointDeg : targetHeadingDeg;
            desiredHeadingDeg = Constants.wrapDeg(desiredHeadingDeg);

            double headingErrDeg = Constants.wrapDeg(desiredHeadingDeg - h);
            double finalHeadingErrDeg = Constants.wrapDeg(targetHeadingDeg - h);

            boolean atPos = dist <= posTolIn;
            boolean atAng = Math.abs(finalHeadingErrDeg) <= angTolDeg;

            if (atPos && atAng) {
                if (settleStart < 0) settleStart = System.currentTimeMillis();
                if (System.currentTimeMillis() - settleStart >= settleMs) break;
            } else {
                settleStart = -1;
            }

            double fwd = Range.clip(dist * kPDist, -maxFwd, maxFwd);

            if (useFacingScale) {
                double facingScale = Math.cos(Math.toRadians(headingErrDeg));
                fwd *= Range.clip(facingScale, 0.0, 1.0);
            }

            if (kSFwd > 0.0 && Math.abs(fwd) > 1e-6) {
                if (Math.abs(fwd) < kSFwd) fwd = kSFwd * Math.signum(fwd);
            }

            double turn = Range.clip((headingErrDeg * kPAng) * turnSign, -maxTurn, maxTurn);

            if (kSTurn > 0.0 && Math.abs(headingErrDeg) > angTolDeg) {
                if (Math.abs(turn) < kSTurn) turn = kSTurn * Math.signum(turn);
            }

            if (atPos && atAng) {
                fwd = 0.0;
                turn = 0.0;
            }

            Drive.INSTANCE.drive(fwd, turn);
            opMode.idle();
        }

        Drive.INSTANCE.stop();
    }

    public void beginAbs(double targetXIn, double targetYIn, double targetHeadingDeg) {
        Drive.INSTANCE.updateOdometry();

        this.targetXAbs = targetXIn;
        this.targetYAbs = targetYIn;
        this.targetHAbs = Constants.wrapDeg(targetHeadingDeg);

        this.busy = true;
        this.startTimeMs = System.currentTimeMillis();
        this.settleStartMs = -1;
    }

    public boolean isBusy() {
        return busy;
    }

    public void cancel() {
        busy = false;
        settleStartMs = -1;
        Drive.INSTANCE.stop();
    }

    public boolean step() {
        if (!busy) return false;

        if (timeoutSec > 0.0) {
            double elapsed = (System.currentTimeMillis() - startTimeMs) / 1000.0;
            if (elapsed >= timeoutSec) {
                cancel();
                return false;
            }
        }

        Drive.INSTANCE.updateOdometry();

        double x = Drive.INSTANCE.getX();
        double y = Drive.INSTANCE.getY();
        double h = Drive.INSTANCE.getHeading();

        double ex = targetXAbs - x;
        double ey = targetYAbs - y;
        double dist = Math.hypot(ex, ey);

        double angleToPointDeg = Math.toDegrees(Math.atan2(ey, ex));

        double desiredHeadingDeg = (dist > headingBlendDistIn) ? angleToPointDeg : targetHAbs;
        desiredHeadingDeg = Constants.wrapDeg(desiredHeadingDeg);

        double headingErrDeg = Constants.wrapDeg(desiredHeadingDeg - h);
        double finalHeadingErrDeg = Constants.wrapDeg(targetHAbs - h);

        boolean atPos = dist <= posTolIn;
        boolean atAng = Math.abs(finalHeadingErrDeg) <= angTolDeg;

        if (atPos && atAng) {
            if (settleStartMs < 0) settleStartMs = System.currentTimeMillis();
            Drive.INSTANCE.drive(0.0, 0.0);
            if (System.currentTimeMillis() - settleStartMs >= settleMs) {
                cancel();
                return false;
            }
            return true;
        } else {
            settleStartMs = -1;
        }

        double fwd = Range.clip(dist * kPDist, -maxFwd, maxFwd);

        if (useFacingScale) {
            double facingScale = Math.cos(Math.toRadians(headingErrDeg));
            fwd *= Range.clip(facingScale, 0.0, 1.0);
        }

        if (kSFwd > 0.0 && Math.abs(fwd) > 1e-6) {
            if (Math.abs(fwd) < kSFwd) fwd = kSFwd * Math.signum(fwd);
        }

        double turn = Range.clip((headingErrDeg * kPAng) * turnSign, -maxTurn, maxTurn);

        if (kSTurn > 0.0 && Math.abs(headingErrDeg) > angTolDeg) {
            if (Math.abs(turn) < kSTurn) turn = kSTurn * Math.signum(turn);
        }

        Drive.INSTANCE.drive(fwd, turn);
        return true;
    }

    public void driveTo(LinearOpMode opMode, double forwardIn) {
        Drive.INSTANCE.updateOdometry();

        double startX = Drive.INSTANCE.getX();
        double startY = Drive.INSTANCE.getY();
        double startH = Drive.INSTANCE.getHeading();

        double hRad = Math.toRadians(startH);

        double targetX = startX + forwardIn * Math.cos(hRad);
        double targetY = startY + forwardIn * Math.sin(hRad);

        goToPose(opMode, targetX, targetY, startH);
    }

    public void turnTo(LinearOpMode opMode, double targetHeadingDeg) {
        Drive.INSTANCE.updateOdometry();
        goToPose(opMode, Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), targetHeadingDeg);
    }

    public void goTo(LinearOpMode opMode, double forwardIn, double leftIn, double endHeadingDeg) {
        Drive.INSTANCE.updateOdometry();

        double startX = Drive.INSTANCE.getX();
        double startY = Drive.INSTANCE.getY();
        double startH = Drive.INSTANCE.getHeading();

        double hRad = Math.toRadians(startH);

        double targetX = startX + forwardIn * Math.cos(hRad) - leftIn * Math.sin(hRad);
        double targetY = startY + forwardIn * Math.sin(hRad) + leftIn * Math.cos(hRad);

        goToPose(opMode, targetX, targetY, endHeadingDeg);
    }

    public void goTo(LinearOpMode opMode, double forwardIn, double leftIn) {
        Drive.INSTANCE.updateOdometry();
        goTo(opMode, forwardIn, leftIn, Drive.INSTANCE.getHeading());
    }
}
