//package SubSystems;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import Util.Constants;
//
//public class Turret {
//    public static final Turret INSTANCE = new Turret();
//
//    private Turret() {
//    }
//
//    private DcMotorEx turretRotation;
//
//    @SuppressWarnings("unused")
//    private Telemetry telemetry;
//
//    // Turret Variables
//    private double turretZeroOffsetTicks = 0.0;
//    private final ElapsedTime turretAimTimer = new ElapsedTime();
//
//    private enum TurretAimMode { Quick, Precise }
//
//    private TurretAimMode aimMode = TurretAimMode.Quick;
//    private double lastTargetSeenTimeS = 0.0;
//
//    public void init(HardwareMap hw, Telemetry telem) {
//        this.telemetry = telem;
//        turretRotation = hw.get(DcMotorEx.class, Constants.Turret.turretRotation);
//
//        turretRotation.setPower(0.0);
//        turretAimTimer.reset();
//        aimMode = TurretAimMode.Quick;
//        lastTargetSeenTimeS = 0.0;
//        turretZeroOffsetTicks = turretRotation.getCurrentPosition();
//    }
//
//    public void zeroTurret() {
//        if (turretRotation == null) return;
//        turretZeroOffsetTicks = turretRotation.getCurrentPosition();
//    }
//
//    public double getTurretDeg() {
//        if (turretRotation == null) return 0.0;
//        double ticks = turretRotation.getCurrentPosition() - turretZeroOffsetTicks;
//        return ticks * Constants.Turret.TurretDegPerTick;
//    }
//
//    private static double clamp(double v, double lo, double hi) {
//        return Math.max(lo, Math.min(hi, v));
//    }
//
//    private static double wrapDeg(double deg) {
//        while (deg > 180.0) deg -= 360.0;
//        while (deg <= -180.0) deg += 360.0;
//        return deg;
//    }
//
//    public String getAimModeName() {
//        return aimMode.name();
//    }
//
//    public boolean atMinLimit() {
//        return getTurretDeg() <= (Constants.Turret.TurretMinDeg + Constants.Turret.LimitGuard);
//    }
//
//    public boolean atMaxLimit() {
//        return getTurretDeg() >= (Constants.Turret.TurretMaxDeg - Constants.Turret.LimitGuard);
//    }
//
//    public double applyTurretLimitsToPower(double requestedPower) {
//        if (requestedPower < 0 && atMinLimit()) return 0.0;
//        if (requestedPower > 0 && atMaxLimit()) return 0.0;
//        return requestedPower;
//    }
//
//    public double turretErrDeg(double desiredDeg, double currentDeg) {
//        double errShort = wrapDeg(desiredDeg - currentDeg);
//        double errLong;
//
//        if (errShort >= 0) {
//            errLong = errShort - 360.0;
//        } else {
//            errLong = errShort + 360.0;
//        }
//
//        double targetShort = currentDeg + errShort;
//        double targetLong = currentDeg + errLong;
//
//        boolean shortOk = (targetShort >= Constants.Turret.TurretMinDeg) && (targetShort <= Constants.Turret.TurretMaxDeg);
//        boolean longOk = (targetLong >= Constants.Turret.TurretMinDeg) && (targetLong <= Constants.Turret.TurretMaxDeg);
//
//        if (shortOk && longOk) {
//            return (Math.abs(errShort) <= Math.abs(errLong)) ? errShort : errLong;
//        }
//        if (shortOk) return errShort;
//        if (longOk) return errLong;
//
//        double clampedTarget = clamp(desiredDeg, Constants.Turret.TurretMinDeg, Constants.Turret.TurretMaxDeg);
//        return clampedTarget - currentDeg;
//    }
//
//    public void setTurretPower(double pwr) {
//        if (turretRotation == null) return;
//        turretRotation.setPower(Range.clip(pwr, -1.0, 1.0));
//    }
//
//    public void stopTurret() {
//        if (turretRotation == null) return;
//        turretRotation.setPower(0.0);
//    }
//
//    public boolean autoAimTurret(double robotHeadingDeg, double goalHeadingDeg, boolean useVisionFineTune) {
//        if (turretRotation == null) return false;
//
//        double nowS = turretAimTimer.seconds();
//        if (Vision.INSTANCE.hasTrackedTag()) lastTargetSeenTimeS = nowS;
//
//        double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
//        double currentTurretDeg = getTurretDeg();
//        double errDeg = turretErrDeg(desiredTurretDeg, currentTurretDeg);
//
//        if (aimMode == TurretAimMode.Quick) {
//            if (useVisionFineTune && Vision.INSTANCE.hasTrackedTag() && Math.abs(errDeg) <= Constants.Turret.SwitchDeadband) {
//                aimMode = TurretAimMode.Precise;
//            }
//        } else {
//            if (!useVisionFineTune) {
//                aimMode = TurretAimMode.Quick;
//            } else {
//                boolean targetRecentlyLost = (!Vision.INSTANCE.hasTrackedTag()) && ((nowS - lastTargetSeenTimeS) > Constants.Turret.LostTargetTimeout);
//                if (targetRecentlyLost) {
//                    aimMode = TurretAimMode.Quick;
//                }
//            }
//        }
//
//        if (aimMode == TurretAimMode.Precise && useVisionFineTune && Vision.INSTANCE.hasTrackedTag()) {
//            if (Math.abs(Vision.INSTANCE.getTrackedYawDeg()) <= Constants.Turret.PreciseDeadband) {
//                stopTurret();
//                return true;
//            }
//            double cmd = Range.clip(Vision.INSTANCE.getTrackedYawDeg() * Constants.Turret.PreciseKp, -Constants.Turret.PreciseMaxPower, Constants.Turret.PreciseMaxPower);
//            cmd = applyTurretLimitsToPower(cmd);
//            setTurretPower(cmd);
//            return false;
//        }
//
//        if (Math.abs(errDeg) <= Constants.Turret.QuickDeadband) {
//            stopTurret();
//            return false;
//        }
//        double cmd = Range.clip(errDeg * Constants.Turret.QuickKp, -Constants.Turret.QuickMaxPower, Constants.Turret.QuickMaxPower);
//        cmd = applyTurretLimitsToPower(cmd);
//        setTurretPower(cmd);
//        return false;
//    }
//}
