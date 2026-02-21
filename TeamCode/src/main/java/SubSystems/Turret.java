package SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Turret {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    private DcMotorEx turretRotation;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    // Turret Variables
    private double turretZeroOffsetTicks = 0.0;
    private final ElapsedTime turretAimTimer = new ElapsedTime();

    private enum TurretAimMode {Quick, Precise}

    private TurretAimMode aimMode = TurretAimMode.Quick;
    private double lastTargetSeenTimeS = 0.0;

    private double turretTargetDeg = 0.0;
    private double turretErrDeg = 0.0;
    private double lastPreciseErrDeg = 0.0;
    private double lastPreciseTimeS = 0.0;

    private static final double FORWARD_LOCK_DEADBAND_DEG = 0.5;
    private static final double FORWARD_LOCK_KP = 0.03;
    private static final double FORWARD_LOCK_MAX_POWER = 0.80;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;
        turretRotation = hw.get(DcMotorEx.class, Constants.Turret.turretRotation);
        turretRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretRotation.setPower(0.0);
        turretAimTimer.reset();
        aimMode = TurretAimMode.Quick;
        lastTargetSeenTimeS = 0.0;

        turretTargetDeg = 0.0;
        turretErrDeg = 0.0;
        lastPreciseErrDeg = 0.0;
        lastPreciseTimeS = 0.0;
        turretZeroOffsetTicks = turretRotation.getCurrentPosition();
    }

    public void zeroTurret() {
        if (turretRotation == null) return;
        turretZeroOffsetTicks = turretRotation.getCurrentPosition();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    public String getAimModeName() {
        return aimMode.name();
    }

    public boolean atMinLimit() {
        return getTurretDeg() <= (Constants.Turret.TurretMinDeg + Constants.Turret.LimitGuard);
    }

    public boolean atMaxLimit() {
        return getTurretDeg() >= (Constants.Turret.TurretMaxDeg - Constants.Turret.LimitGuard);
    }

    public double applyTurretLimitsToPower(double requestedPower) {
        if (requestedPower < 0 && atMinLimit()) return 0.0;
        if (requestedPower > 0 && atMaxLimit()) return 0.0;
        return requestedPower;
    }

    public double getTurretDeg() {
        if (turretRotation == null) return 0.0;
        double ticks = turretRotation.getCurrentPosition() - turretZeroOffsetTicks;
        return ticks * Constants.Turret.TurretDegPerTick;
    }

    public double getTargetDeg() {
        return turretTargetDeg;
    }

    public double getErrorDeg() {
        return turretErrDeg;
    }

    public double turretErrDeg(double desiredDeg, double currentDeg) {
        double errShort = wrapDeg(desiredDeg - currentDeg);
        double errLong;

        if (errShort >= 0) {
            errLong = errShort - 360.0;
        } else {
            errLong = errShort + 360.0;
        }

        double targetShort = currentDeg + errShort;
        double targetLong = currentDeg + errLong;

        boolean shortOk = (targetShort >= Constants.Turret.TurretMinDeg) && (targetShort <= Constants.Turret.TurretMaxDeg);
        boolean longOk = (targetLong >= Constants.Turret.TurretMinDeg) && (targetLong <= Constants.Turret.TurretMaxDeg);

        if (shortOk && longOk) {
            return (Math.abs(errShort) <= Math.abs(errLong)) ? errShort : errLong;
        }
        if (shortOk) return errShort;
        if (longOk) return errLong;

        double clampedTarget = clamp(desiredDeg, Constants.Turret.TurretMinDeg, Constants.Turret.TurretMaxDeg);
        return clampedTarget - currentDeg;
    }

    public void setTurretPower(double pwr) {
        if (turretRotation == null) return;
        turretRotation.setPower(Range.clip(pwr, -1.0, 1.0));
    }

    public void stopTurret() {
        if (turretRotation == null) return;
        turretRotation.setPower(0.0);
    }

    public boolean lockTurret() {
        if (turretRotation == null) return false;

        double errDeg = -getTurretDeg();
        turretTargetDeg = 0.0;
        turretErrDeg = errDeg;

        if (Math.abs(errDeg) <= FORWARD_LOCK_DEADBAND_DEG) {
            stopTurret();
            return true;
        }

        double cmd = Range.clip(errDeg * FORWARD_LOCK_KP, -FORWARD_LOCK_MAX_POWER, FORWARD_LOCK_MAX_POWER);
        cmd = applyTurretLimitsToPower(cmd);

        if (Math.abs(cmd) < 1e-6) stopTurret();
        else setTurretPower(cmd);

        return false;
    }

    public boolean autoAimTurret(double robotHeadingDeg, double goalHeadingDeg) {
        if (turretRotation == null) return false;

        double nowS = turretAimTimer.seconds();
        if (Vision.INSTANCE.hasTrackedTag()) lastTargetSeenTimeS = nowS;

        double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
        double currentTurretDeg = getTurretDeg();
        double errDeg = turretErrDeg(desiredTurretDeg, currentTurretDeg);
        turretTargetDeg = desiredTurretDeg;
        turretErrDeg = errDeg;

        if (aimMode == TurretAimMode.Quick) {
            if (Vision.INSTANCE.hasTrackedTag() && Math.abs(errDeg) <= Constants.Turret.SwitchDeadband) {
                aimMode = TurretAimMode.Precise;
            }
        } else {
            boolean targetRecentlyLost = (!Vision.INSTANCE.hasTrackedTag()) && ((nowS - lastTargetSeenTimeS) > Constants.Turret.LostTargetTimeout);
            if (targetRecentlyLost) {
                aimMode = TurretAimMode.Quick;
            }
        }

        if (aimMode == TurretAimMode.Precise && Vision.INSTANCE.hasTrackedTag()) {

            double tagErrDeg = Vision.INSTANCE.getTrackedTagCX();  // error in degrees (pixel->deg)
            double now = nowS;

            double dt = (lastPreciseTimeS > 0.0) ? (now - lastPreciseTimeS) : 0.0;
            if (dt <= 1e-6) dt = 0.02;

            double errRate = (tagErrDeg - lastPreciseErrDeg) / dt;

            lastPreciseErrDeg = tagErrDeg;
            lastPreciseTimeS = now;

            boolean inPos = Math.abs(tagErrDeg) <= Constants.Turret.PreciseDeadband;
            boolean slow = Math.abs(errRate) <= Constants.Turret.PreciseRateDeadband;

            if (inPos && slow) {
                stopTurret();
                return true;
            }

            double requested = (tagErrDeg * Constants.Turret.PreciseKp) - (errRate * Constants.Turret.PreciseKd);
            double cmd = Range.clip(requested, -Constants.Turret.PreciseMaxPower, Constants.Turret.PreciseMaxPower);

            if (!inPos && Math.abs(cmd) < Constants.Turret.PreciseMinPower) {
                cmd = Math.copySign(Constants.Turret.PreciseMinPower, cmd);
            }

            cmd = applyTurretLimitsToPower(cmd);

            if (Math.abs(cmd) < 1e-6) stopTurret();
            else setTurretPower(cmd);

            return false;
        }

        if (Math.abs(errDeg) <= Constants.Turret.QuickDeadband) {
            stopTurret();
            return false;
        }

        double cmd = Range.clip(errDeg * Constants.Turret.QuickKp, -Constants.Turret.QuickMaxPower, Constants.Turret.QuickMaxPower);
        cmd = applyTurretLimitsToPower(cmd);

        if (Math.abs(cmd) < 1e-6) stopTurret();
        else setTurretPower(cmd);

        return false;
    }

    public boolean autoAimTurretTunable(double robotHeadingDeg, double goalHeadingDeg, double quickKp, double quickMaxPower, double preciseKp, double preciseMaxPower, double preciseKd, double preciseMinPower, double preciseRateDeadband, boolean forcePrecise) {
        if (turretRotation == null) return false;

        double nowS = turretAimTimer.seconds();
        if (Vision.INSTANCE.hasTrackedTag()) lastTargetSeenTimeS = nowS;

        double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
        double currentTurretDeg = getTurretDeg();
        double errDeg = turretErrDeg(desiredTurretDeg, currentTurretDeg);

        turretTargetDeg = desiredTurretDeg;
        turretErrDeg = errDeg;

        if (!forcePrecise) {
            if (aimMode == TurretAimMode.Quick) {
                if (Vision.INSTANCE.hasTrackedTag() && Math.abs(errDeg) <= Constants.Turret.SwitchDeadband) {
                    aimMode = TurretAimMode.Precise;
                    lastPreciseErrDeg = 0.0;
                    lastPreciseTimeS = 0.0;
                }
            } else {
                boolean targetRecentlyLost = (!Vision.INSTANCE.hasTrackedTag()) &&
                        ((nowS - lastTargetSeenTimeS) > Constants.Turret.LostTargetTimeout);
                if (targetRecentlyLost) {
                    aimMode = TurretAimMode.Quick;
                    lastPreciseErrDeg = 0.0;
                    lastPreciseTimeS = 0.0;
                }
            }
        }

        boolean doPrecise = forcePrecise || (aimMode == TurretAimMode.Precise);

        if (doPrecise) {
            if (!Vision.INSTANCE.hasTrackedTag()) {
                stopTurret();
                lastPreciseErrDeg = 0.0;
                lastPreciseTimeS = 0.0;
                return false;
            }

            double tagErrDeg = Vision.INSTANCE.getTrackedTagCX();

            double dt = (lastPreciseTimeS > 0.0) ? (nowS - lastPreciseTimeS) : 0.0;
            if (dt <= 1e-6) dt = 0.02;

            double errRate = (tagErrDeg - lastPreciseErrDeg) / dt;

            lastPreciseErrDeg = tagErrDeg;
            lastPreciseTimeS = nowS;

            boolean inPos = Math.abs(tagErrDeg) <= Constants.Turret.PreciseDeadband;
            boolean slow = Math.abs(errRate) <= preciseRateDeadband;

            if (inPos && slow) {
                stopTurret();
                return true;
            }

            double requested = (tagErrDeg * preciseKp) - (errRate * preciseKd);
            double cmd = Range.clip(requested, -preciseMaxPower, preciseMaxPower);

            if (!inPos && Math.abs(cmd) < preciseMinPower) {
                cmd = Math.copySign(preciseMinPower, cmd);
            }

            cmd = applyTurretLimitsToPower(cmd);

            if (Math.abs(cmd) < 1e-6) stopTurret();
            else setTurretPower(cmd);

            return false;
        }

        if (Math.abs(errDeg) <= Constants.Turret.QuickDeadband) {
            stopTurret();
            return true;
        }

        double cmd = Range.clip(errDeg * quickKp, -quickMaxPower, quickMaxPower);
        cmd = applyTurretLimitsToPower(cmd);

        if (Math.abs(cmd) < 1e-6) stopTurret();
        else setTurretPower(cmd);

        return false;
    }
}