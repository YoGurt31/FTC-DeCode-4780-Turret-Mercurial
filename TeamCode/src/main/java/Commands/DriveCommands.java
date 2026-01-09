package Commands;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.DoubleSupplier;

import SubSystems.DriveSystem;
import dev.frozenmilk.mercurial.commands.Lambda;

public final class DriveCommands {

    private DriveCommands() {}

    public static final class Params {
        public double driveKp = 0.05;
        public double minDrive = 0.10;
        public double maxDrive = 0.80;

        public double headingKp = 0.025;
        public double maxRotate = 0.50;

        public double posToleranceIn = 0.75;
        public double headingToleranceDeg = 1.0;
        public long settleMs = 150;

        public double maxTimeSec = 0.0;

        public Params() {}
    }

    @NonNull
    public static Lambda teleOpDrive(@NonNull DoubleSupplier drive, @NonNull DoubleSupplier rotate) {
        return new Lambda("TeleOpDrive")
                .addRequirements(DriveSystem.INSTANCE)
                .setExecute(() -> {
                    DriveSystem.updatePinpoint();
                    double d = Range.clip(drive.getAsDouble(), -1.0, 1.0);
                    double r = Range.clip(rotate.getAsDouble(), -1.0, 1.0);
                    DriveSystem.tankDrive(d, r);
                })
                .setFinish(() -> false)
                .setEnd(interrupted -> DriveSystem.stop());
    }

    @NonNull
    public static Lambda stop() {
        return new Lambda("DriveStop")
                .addRequirements(DriveSystem.INSTANCE)
                .setInit(DriveSystem::stop)
                .setExecute(DriveSystem::stop)
                .setFinish(() -> true)
                .setEnd(interrupted -> DriveSystem.stop());
    }

    @NonNull
    public static Lambda driveDistance(double distanceIn, double power, @NonNull Params p) {
        final double absPower = Math.abs(power);

        return new Lambda("DriveDistance(" + distanceIn + ")")
                .addRequirements(DriveSystem.INSTANCE)
                .setInit(() -> {
                    DriveSystem.updatePinpoint();
                    State.sx = DriveSystem.pose().getX(DistanceUnit.INCH);
                    State.sy = DriveSystem.pose().getY(DistanceUnit.INCH);
                    State.sh = DriveSystem.pose().getHeading(AngleUnit.DEGREES);
                    State.targetDist = Math.abs(distanceIn);
                    State.dir = Math.signum(distanceIn);
                    if (State.dir == 0.0) State.dir = 1.0;
                    State.settleStart = -1;
                    State.startNs = System.nanoTime();
                })
                .setExecute(() -> {
                    DriveSystem.updatePinpoint();
                    Pose2D cur = DriveSystem.pose();

                    double x = cur.getX(DistanceUnit.INCH);
                    double y = cur.getY(DistanceUnit.INCH);
                    double h = cur.getHeading(AngleUnit.DEGREES);

                    double traveled = Math.hypot(x - State.sx, y - State.sy);
                    double remaining = State.targetDist - traveled;

                    double driveCmd = absPower * State.dir;
                    double headingErr = DriveSystem.wrapDeg(State.sh - h);
                    double rotateCmd = Range.clip(-headingErr * p.headingKp, -p.maxRotate, p.maxRotate);

                    if (remaining <= p.posToleranceIn) {
                        DriveSystem.stop();
                        return;
                    }

                    DriveSystem.tankDrive(driveCmd, rotateCmd);
                })
                .setFinish(() -> {
                    DriveSystem.updatePinpoint();
                    Pose2D cur = DriveSystem.pose();
                    double x = cur.getX(DistanceUnit.INCH);
                    double y = cur.getY(DistanceUnit.INCH);
                    double traveled = Math.hypot(x - State.sx, y - State.sy);

                    boolean distOk = (State.targetDist - traveled) <= p.posToleranceIn;

                    if (p.maxTimeSec > 0.0) {
                        double t = (System.nanoTime() - State.startNs) / 1e9;
                        if (t >= p.maxTimeSec) return true;
                    }

                    if (!distOk) {
                        State.settleStart = -1;
                        return false;
                    }

                    long now = System.currentTimeMillis();
                    if (State.settleStart < 0) State.settleStart = now;
                    return (now - State.settleStart) >= p.settleMs;
                })
                .setEnd(interrupted -> DriveSystem.stop());
    }

    @NonNull
    public static Lambda turnTo(double targetHeadingDeg, @NonNull Params p) {
        return new Lambda("TurnTo(" + targetHeadingDeg + ")")
                .addRequirements(DriveSystem.INSTANCE)
                .setInit(() -> {
                    State.targetHeading = normalize180(targetHeadingDeg);
                    State.settleStart = -1;
                    State.startNs = System.nanoTime();
                })
                .setExecute(() -> {
                    DriveSystem.updatePinpoint();
                    double cur = DriveSystem.pose().getHeading(AngleUnit.DEGREES);
                    double err = DriveSystem.wrapDeg(State.targetHeading - cur);

                    double rotate = Range.clip(-err * p.headingKp, -p.maxRotate, p.maxRotate);

                    double absErr = Math.abs(err);
                    if (absErr > p.headingToleranceDeg && Math.abs(rotate) < 0.08) {
                        rotate = 0.08 * Math.signum(rotate == 0.0 ? -err : rotate);
                    }

                    if (absErr <= p.headingToleranceDeg) {
                        DriveSystem.stop();
                        return;
                    }

                    DriveSystem.tankDrive(0.0, rotate);
                })
                .setFinish(() -> {
                    DriveSystem.updatePinpoint();
                    double cur = DriveSystem.pose().getHeading(AngleUnit.DEGREES);
                    double err = Math.abs(DriveSystem.wrapDeg(State.targetHeading - cur));

                    if (p.maxTimeSec > 0.0) {
                        double t = (System.nanoTime() - State.startNs) / 1e9;
                        if (t >= p.maxTimeSec) return true;
                    }

                    if (err > p.headingToleranceDeg) {
                        State.settleStart = -1;
                        return false;
                    }

                    long now = System.currentTimeMillis();
                    if (State.settleStart < 0) State.settleStart = now;
                    return (now - State.settleStart) >= p.settleMs;
                })
                .setEnd(interrupted -> DriveSystem.stop());
    }

    @NonNull
    public static Lambda driveToXYIn(double targetXIn, double targetYIn, @NonNull Params p) {
        return new Lambda("DriveToXY(" + targetXIn + "," + targetYIn + ")")
                .addRequirements(DriveSystem.INSTANCE)
                .setInit(() -> {
                    DriveSystem.updatePinpoint();
                    Pose2D cur = DriveSystem.pose();
                    State.targetX = targetXIn;
                    State.targetY = targetYIn;
                    State.targetHeading = cur.getHeading(AngleUnit.DEGREES);
                    State.settleStart = -1;
                    State.startNs = System.nanoTime();
                })
                .setExecute(() -> {
                    DriveSystem.updatePinpoint();
                    Pose2D cur = DriveSystem.pose();

                    double x = cur.getX(DistanceUnit.INCH);
                    double y = cur.getY(DistanceUnit.INCH);
                    double h = cur.getHeading(AngleUnit.DEGREES);

                    double dx = State.targetX - x;
                    double dy = State.targetY - y;
                    double dist = Math.hypot(dx, dy);

                    double targetBearing = Math.toDegrees(Math.atan2(dy, dx));
                    double headingErr = DriveSystem.wrapDeg(targetBearing - h);

                    if (dist <= p.posToleranceIn) {
                        DriveSystem.stop();
                        return;
                    }

                    double driveSign = (Math.abs(headingErr) > 90.0) ? -1.0 : 1.0;
                    double drive = driveSign * Range.clip(dist * p.driveKp, p.minDrive, p.maxDrive);

                    if (driveSign < 0.0) {
                        headingErr = DriveSystem.wrapDeg(headingErr + 180.0);
                    }

                    double rotate = Range.clip(-headingErr * p.headingKp, -p.maxRotate, p.maxRotate);

                    DriveSystem.tankDrive(drive, rotate);
                })
                .setFinish(() -> {
                    DriveSystem.updatePinpoint();
                    Pose2D cur = DriveSystem.pose();
                    double x = cur.getX(DistanceUnit.INCH);
                    double y = cur.getY(DistanceUnit.INCH);
                    double dist = Math.hypot(State.targetX - x, State.targetY - y);

                    if (p.maxTimeSec > 0.0) {
                        double t = (System.nanoTime() - State.startNs) / 1e9;
                        if (t >= p.maxTimeSec) return true;
                    }

                    if (dist > p.posToleranceIn) {
                        State.settleStart = -1;
                        return false;
                    }

                    long now = System.currentTimeMillis();
                    if (State.settleStart < 0) State.settleStart = now;
                    return (now - State.settleStart) >= p.settleMs;
                })
                .setEnd(interrupted -> DriveSystem.stop());
    }

    private static double normalize180(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    private static final class State {
        static double sx, sy, sh;
        static double targetDist, dir;

        static double targetX, targetY;
        static double targetHeading;

        static long settleStart;
        static long startNs;
    }
}