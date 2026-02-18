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

    private double lastQuickCmd = 0.0;
    private double lastRobotHeadingDeg = 0.0;
    private double lastRobotHeadingTimeS = 0.0;

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
        turretZeroOffsetTicks = turretRotation.getCurrentPosition();

        lastQuickCmd = 0.0;
        lastRobotHeadingDeg = 0.0;
        lastRobotHeadingTimeS = 0.0;
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

    private double computeRobotHeadingRateDegPerSec(double robotHeadingDeg, double nowS) {
        if (lastRobotHeadingTimeS <= 0.0) {
            lastRobotHeadingTimeS = nowS;
            lastRobotHeadingDeg = robotHeadingDeg;
            return 0.0;
        }

        double dt = nowS - lastRobotHeadingTimeS;
        if (dt <= 1e-6) return 0.0;

        double dHeading = wrapDeg(robotHeadingDeg - lastRobotHeadingDeg);
        lastRobotHeadingDeg = robotHeadingDeg;
        lastRobotHeadingTimeS = nowS;

        return dHeading / dt;
    }

    private double shapeQuickCmd(double requestedCmd, double maxPower, double nowS) {
        double cmd = Range.clip(requestedCmd, -maxPower, maxPower);
        if (Math.abs(cmd) < Constants.Turret.QuickMinPower) cmd = 0.0;
        if (Constants.Turret.QuickSlewPerSec > 0.0) {
            double dt = (lastRobotHeadingTimeS > 0.0) ? (nowS - lastRobotHeadingTimeS) : 0.0;
            if (dt <= 1e-6) dt = 0.02;
            double maxDelta = Constants.Turret.QuickSlewPerSec * dt;
            double delta = cmd - lastQuickCmd;
            if (delta > maxDelta) cmd = lastQuickCmd + maxDelta;
            else if (delta < -maxDelta) cmd = lastQuickCmd - maxDelta;
        }
        lastQuickCmd = cmd;
        return cmd;
    }

    private double shapeQuickCmd(double requestedCmd, double maxPower, double nowS, double minPower, double slewPerSec) {
        double cmd = Range.clip(requestedCmd, -maxPower, maxPower);

        if (Math.abs(cmd) < minPower) cmd = 0.0;

        if (slewPerSec > 0.0) {
            double dt = (lastRobotHeadingTimeS > 0.0) ? (nowS - lastRobotHeadingTimeS) : 0.0;
            if (dt <= 1e-6) dt = 0.02;

            double maxDelta = slewPerSec * dt;
            double delta = cmd - lastQuickCmd;

            if (delta > maxDelta) cmd = lastQuickCmd + maxDelta;
            else if (delta < -maxDelta) cmd = lastQuickCmd - maxDelta;
        }

        lastQuickCmd = cmd;
        return cmd;
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
            if (Math.abs(Vision.INSTANCE.getTrackedYawDeg()) <= Constants.Turret.PreciseDeadband) {
                stopTurret();
                return true;
            }
            double cmd = Range.clip(Vision.INSTANCE.getTrackedYawDeg() * Constants.Turret.PreciseKp, -Constants.Turret.PreciseMaxPower, Constants.Turret.PreciseMaxPower);
            cmd = applyTurretLimitsToPower(cmd);
            setTurretPower(cmd);
            return false;
        }

        if (Math.abs(errDeg) <= Constants.Turret.QuickDeadband) {
            stopTurret();
            lastQuickCmd = 0.0;
            return false;
        }

        double robotHeadingRate = computeRobotHeadingRateDegPerSec(robotHeadingDeg, nowS);
        double kF = -Constants.Turret.QuickKfTurnRate * robotHeadingRate;

        double requested = (errDeg * Constants.Turret.QuickKp) + kF;
        double cmd = shapeQuickCmd(requested, Constants.Turret.QuickMaxPower, nowS);
        cmd = applyTurretLimitsToPower(cmd);

        if (Math.abs(cmd) < 1e-6) stopTurret();
        else setTurretPower(cmd);
        return false;
    }

    public boolean autoAimTurretTunable(double robotHeadingDeg, double goalHeadingDeg, double quickKp, double quickMaxPower, double quickMinPower, double quickSlewPerSec, double quickKfTurnRate, double preciseKp, double preciseMaxPower, boolean forcePrecise) {
        if (turretRotation == null) return false;

        double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
        double currentTurretDeg = getTurretDeg();
        double errDeg = turretErrDeg(desiredTurretDeg, currentTurretDeg);

        turretTargetDeg = desiredTurretDeg;
        turretErrDeg = errDeg;

        if (forcePrecise) {
            if (!Vision.INSTANCE.hasTrackedTag()) {
                stopTurret();
                lastQuickCmd = 0.0;
                return false;
            }

            double yaw = Vision.INSTANCE.getTrackedYawDeg();

            if (Math.abs(yaw) <= Constants.Turret.PreciseDeadband) {
                stopTurret();
                lastQuickCmd = 0.0;
                return true;
            }

            double cmd = Range.clip(yaw * preciseKp, -preciseMaxPower, preciseMaxPower);
            cmd = applyTurretLimitsToPower(cmd);
            setTurretPower(cmd);
            return false;
        }

        if (Math.abs(errDeg) <= Constants.Turret.QuickDeadband) {
            stopTurret();
            lastQuickCmd = 0.0;
            return false;
        }

        double nowS = turretAimTimer.seconds();
        double robotHeadingRate = computeRobotHeadingRateDegPerSec(robotHeadingDeg, nowS);
        double kF = -quickKfTurnRate * robotHeadingRate;

        double requested = (errDeg * quickKp) + kF;
        double cmd = shapeQuickCmd(requested, quickMaxPower, nowS, quickMinPower, quickSlewPerSec);
        cmd = applyTurretLimitsToPower(cmd);

        if (Math.abs(cmd) < 1e-6) stopTurret();
        else setTurretPower(cmd);
        return false;
    }
}