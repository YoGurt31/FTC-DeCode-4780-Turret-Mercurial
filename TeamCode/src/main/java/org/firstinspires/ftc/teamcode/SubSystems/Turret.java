package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Util.Constants;

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

    private double turretTargetDeg = 0.0;
    private double turretErrDeg = 0.0;

    private static final double FORWARD_LOCK_DEADBAND_DEG = 0.5;
    private static final double FORWARD_LOCK_KP = 0.025;
    private static final double FORWARD_LOCK_MAX_POWER = 0.75;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;
        turretRotation = hw.get(DcMotorEx.class, Constants.Turret.TURRET_ROTATION);
        turretRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretRotation.setPower(0.0);
        turretAimTimer.reset();

        turretTargetDeg = 0.0;
        turretErrDeg = 0.0;
    }

    public void zeroTurret() {
        if (turretRotation == null) return;
        turretZeroOffsetTicks = turretRotation.getCurrentPosition();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public boolean atMinLimit() {
        return getTurretDeg() <= (Constants.Turret.TURRET_MIN_DEG + Constants.Turret.LIMIT_GUARD);
    }

    public boolean atMaxLimit() {
        return getTurretDeg() >= (Constants.Turret.TURRET_MAX_DEG - Constants.Turret.LIMIT_GUARD);
    }

    public double applyTurretLimitsToPower(double requestedPower) {
        if (requestedPower < 0 && atMinLimit()) return 0.0;
        if (requestedPower > 0 && atMaxLimit()) return 0.0;
        return requestedPower;
    }

    public double getTurretDeg() {
        if (turretRotation == null) return 0.0;
        double ticks = turretRotation.getCurrentPosition() - turretZeroOffsetTicks;
        return ticks * Constants.Turret.TURRET_DEG_PER_TICK + Constants.Turret.TURRET_HOME_DEG;
    }

    public double getTargetDeg() {
        return turretTargetDeg;
    }

    public double getErrorDeg() {
        return turretErrDeg;
    }

    public boolean isAligned() {
        return Math.abs(Turret.INSTANCE.getErrorDeg()) <= 1.0;
    }

    public double turretErrDeg(double desiredDeg, double currentDeg) {
        double errShort = Constants.wrapDeg(desiredDeg - currentDeg);
        double errLong;

        if (errShort >= 0) {
            errLong = errShort - 360.0;
        } else {
            errLong = errShort + 360.0;
        }

        double targetShort = currentDeg + errShort;
        double targetLong = currentDeg + errLong;

        boolean shortOk = (targetShort >= Constants.Turret.TURRET_MIN_DEG) && (targetShort <= Constants.Turret.TURRET_MAX_DEG);
        boolean longOk = (targetLong >= Constants.Turret.TURRET_MIN_DEG) && (targetLong <= Constants.Turret.TURRET_MAX_DEG);

        if (shortOk && longOk) {
            return (Math.abs(errShort) <= Math.abs(errLong)) ? errShort : errLong;
        }
        if (shortOk) return errShort;
        if (longOk) return errLong;

        double clampedTarget = clamp(desiredDeg, Constants.Turret.TURRET_MIN_DEG, Constants.Turret.TURRET_MAX_DEG);
        return clampedTarget - currentDeg;
    }

    public void setTurretPower(double pwr) {
        if (turretRotation == null) return;
        turretRotation.setPower(Range.clip(pwr, -1.0, 1.0));
    }

    public void stop() {
        if (turretRotation == null) return;
        turretRotation.setPower(0.0);
    }

    public boolean lockTurret() {
        if (turretRotation == null) return false;

        double errDeg = turretErrDeg(Constants.Turret.TURRET_HOME_DEG, getTurretDeg());
        turretTargetDeg = Constants.Turret.TURRET_HOME_DEG;
        turretErrDeg = errDeg;

        if (Math.abs(errDeg) <= FORWARD_LOCK_DEADBAND_DEG) {
            stop();
            return true;
        }

        double cmd = Range.clip(errDeg * FORWARD_LOCK_KP, -FORWARD_LOCK_MAX_POWER, FORWARD_LOCK_MAX_POWER);
        cmd = applyTurretLimitsToPower(cmd);

        setTurretPower(cmd);
        return false;
    }

    public boolean lockTurretAt(double deg) {
        if (turretRotation == null) return false;

        double targetDeg = Constants.wrapDeg(deg);
        targetDeg = clamp(targetDeg, Constants.Turret.TURRET_MIN_DEG, Constants.Turret.TURRET_MAX_DEG);
        double currentDeg = getTurretDeg();

        double errDeg = turretErrDeg(targetDeg, currentDeg);
        turretTargetDeg = targetDeg;
        turretErrDeg = errDeg;

        if (Math.abs(errDeg) <= FORWARD_LOCK_DEADBAND_DEG) {
            stop();
            return true;
        }

        double cmd = Range.clip(errDeg * FORWARD_LOCK_KP, -FORWARD_LOCK_MAX_POWER, FORWARD_LOCK_MAX_POWER);
        cmd = applyTurretLimitsToPower(cmd);

        setTurretPower(cmd);
        return false;
    }

    public boolean autoAimTurret(double robotHeadingDeg, double goalHeadingDeg) {
        if (turretRotation == null) return false;

        double desiredTurretDeg = Constants.wrapDeg(goalHeadingDeg - robotHeadingDeg);
        double currentTurretDeg = getTurretDeg();
        double errDeg = turretErrDeg(desiredTurretDeg, currentTurretDeg);

        turretTargetDeg = desiredTurretDeg;
        turretErrDeg = errDeg;

        if (Math.abs(errDeg) <= Constants.Turret.TURRET_DEADBAND) {
            stop();
            return true;
        }

        double cruise1 = 0.0, cruise2 = 0.0;
        double cmd = (Constants.Turret.TURRET_KP * errDeg) + (Constants.Turret.TURRET_KS * Math.signum(errDeg));
        if (Math.abs(errDeg) > Constants.Turret.TURRET_CRUISE1_DEADBAND) cruise1 = Constants.Turret.TURRET_CRUISE1_POWER;
        cmd = Math.signum(errDeg) * Math.max(Math.abs(cmd), cruise1);
        if (Math.abs(errDeg) < Constants.Turret.TURRET_CRUISE1_DEADBAND && Math.abs(errDeg) > Constants.Turret.TURRET_CRUISE2_DEADBAND) cruise2 = Constants.Turret.TURRET_CRUISE2_POWER;
        cmd = Math.signum(errDeg) * Math.max(Math.abs(cmd), cruise2);

        cmd = Range.clip(cmd, -Constants.Turret.TURRET_MAX_POWER, +Constants.Turret.TURRET_MAX_POWER);
        cmd = applyTurretLimitsToPower(cmd);

        setTurretPower(cmd);
        return false;
    }
}