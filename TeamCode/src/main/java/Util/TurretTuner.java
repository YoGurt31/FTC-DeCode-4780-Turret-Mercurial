package Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.qualcomm.robotcore.util.Range;

import SubSystems.Drive;
import SubSystems.Turret;
import SubSystems.Vision;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
public final class TurretTuner {

    private static boolean rising(boolean now, boolean[] prev) {
        boolean r = now && !prev[0];
        prev[0] = now;
        return r;
    }

    public static final Mercurial.RegisterableProgram TurretTUNER = Mercurial.teleop(linsane -> {

        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        final Constants.Field.Alliance[] alliance = { Constants.Field.Alliance.RED };
        Vision.INSTANCE.setTrackedTag(alliance[0] == Constants.Field.Alliance.RED
                ? Constants.Vision.RED_TAG_ID
                : Constants.Vision.BLUE_TAG_ID
        );

        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        final double[] steps = { 0.001, 0.0025, 0.005, 0.01, 0.02, 0.05, 0.1, 0.25, 0.5, 1.0 };
        final int[] stepIdx = { 3 };

        final long[] lastAdjustMs = { 0 };
        final long repeatMs = 120;

        final double[] quickKp = { Constants.Turret.QuickKp };
        final double[] quickMax = { Constants.Turret.QuickMaxPower };

        final double[] preciseKp = { Constants.Turret.PreciseKp };
        final double[] preciseMax = { Constants.Turret.PreciseMaxPower };

        final double[] preciseKd = { Constants.Turret.PreciseKd };
        final double[] preciseMin = { Constants.Turret.PreciseMinPower };
        final double[] preciseRateDb = { Constants.Turret.PreciseRateDeadband };

        final int[] turretSel = { 0 };
        final boolean[] turretPrecise = { false };

        final boolean[] prevDL = { false }, prevDR = { false }, prevLB = { false }, prevRB = { false };
        final boolean[] prevLS = { false }, prevRS = { false };

        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                loop(exec(() -> {

                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();

                    if (rising(linsane.gamepad1().dpad_left, prevDL)) stepIdx[0] = Math.max(0, stepIdx[0] - 1);
                    if (rising(linsane.gamepad1().dpad_right, prevDR)) stepIdx[0] = Math.min(steps.length - 1, stepIdx[0] + 1);
                    double step = steps[stepIdx[0]];

                    double driveCmd = -linsane.gamepad1().left_stick_y;
                    double turnCmd = linsane.gamepad1().right_stick_x;
                    Drive.INSTANCE.drive(driveCmd, turnCmd);

                    long now = System.currentTimeMillis();
                    boolean doAdjust = false;
                    int dir = 0;

                    if (linsane.gamepad1().dpad_up) {
                        dir = +1;
                        doAdjust = (now - lastAdjustMs[0]) >= repeatMs;
                    } else if (linsane.gamepad1().dpad_down) {
                        dir = -1;
                        doAdjust = (now - lastAdjustMs[0]) >= repeatMs;
                    } else {
                        lastAdjustMs[0] = 0;
                    }
                    if (doAdjust) lastAdjustMs[0] = now;

                    int tagId = (alliance[0] == Constants.Field.Alliance.RED)
                            ? Constants.Vision.RED_TAG_ID
                            : Constants.Vision.BLUE_TAG_ID;
                    Vision.INSTANCE.setTrackedTag(tagId);

                    if (rising(linsane.gamepad1().right_stick_button, prevRS)) {
                        turretPrecise[0] = !turretPrecise[0];
                    }

                    if (rising(linsane.gamepad1().left_stick_button, prevLS)) {
                        alliance[0] = (alliance[0] == Constants.Field.Alliance.RED)
                                ? Constants.Field.Alliance.BLUE
                                : Constants.Field.Alliance.RED;
                        tagId = (alliance[0] == Constants.Field.Alliance.RED)
                                ? Constants.Vision.RED_TAG_ID
                                : Constants.Vision.BLUE_TAG_ID;
                        Vision.INSTANCE.setTrackedTag(tagId);
                    }

                    if (rising(linsane.gamepad1().left_bumper, prevLB)) turretSel[0] = (turretSel[0] + 6) % 7;
                    if (rising(linsane.gamepad1().right_bumper, prevRB)) turretSel[0] = (turretSel[0] + 1) % 7;

                    if (doAdjust) {
                        switch (turretSel[0]) {
                            case 0: quickKp[0] = Math.max(0.0, quickKp[0] + dir * step); break;
                            case 1: quickMax[0] = Range.clip(quickMax[0] + dir * step, 0.0, 1.0); break;

                            case 2: preciseKp[0] = Math.max(0.0, preciseKp[0] + dir * step); break;
                            case 3: preciseMax[0] = Range.clip(preciseMax[0] + dir * step, 0.0, 1.0); break;
                            case 4: preciseKd[0] = Math.max(0.0, preciseKd[0] + dir * step); break;
                            case 5: preciseMin[0] = Range.clip(preciseMin[0] + dir * step, 0.0, 1.0); break;
                            case 6: preciseRateDb[0] = Math.max(0.0, preciseRateDb[0] + dir * step); break;
                        }
                    }

                    double robotX = Drive.INSTANCE.getX();
                    double robotY = Drive.INSTANCE.getY();
                    double robotHeading = Drive.INSTANCE.getHeading();
                    double goalHeading = Constants.Field.computeGoalHeadingDeg(robotX, robotY, alliance[0]);

                    boolean atTarget = Turret.INSTANCE.autoAimTurretTunable(
                            robotHeading, goalHeading,
                            quickKp[0], quickMax[0],
                            preciseKp[0], preciseMax[0],
                            preciseKd[0], preciseMin[0], preciseRateDb[0],
                            turretPrecise[0]
                    );

                    double turretDeg = Turret.INSTANCE.getTurretDeg();
                    double desiredTurretDeg = Turret.INSTANCE.getTargetDeg();
                    double errDeg = Turret.INSTANCE.getErrorDeg();

                    boolean usingTag = turretPrecise[0] && Vision.INSTANCE.hasTrackedTag();
                    double yawDeg = usingTag ? Vision.INSTANCE.getTrackedTagCX() : 0.0;

                    double centerX = Double.NaN;
                    double errPx = Double.NaN;
                    double degPerPx = Double.NaN;
                    try {
                        centerX = Vision.INSTANCE.getTrackedCenterX();
                        errPx = Vision.INSTANCE.getTrackedCenterError();
                        degPerPx = Constants.Vision.TURRET_DEG_PER_PIXEL;
                    } catch (Throwable ignored) { }

                    linsane.telemetry().addLine("=== Turret Tuner ===");
                    linsane.telemetry().addLine("R3 toggle Quick/Precise | L3 toggle Alliance | LB/RB cycle param");
                    linsane.telemetry().addLine("Dpad L/R step size | Dpad U/D adjust selected param");
                    linsane.telemetry().addData("Alliance", alliance[0]);
                    linsane.telemetry().addData("TagId", (alliance[0] == Constants.Field.Alliance.RED)
                            ? Constants.Vision.RED_TAG_ID : Constants.Vision.BLUE_TAG_ID);
                    linsane.telemetry().addData("Mode", turretPrecise[0] ? "PRECISE (Cam)" : "QUICK (PinPoint)");
                    linsane.telemetry().addData("AtTarget", atTarget);
                    linsane.telemetry().addData("Step", step);

                    String sel = new String[]{
                            "QuickKp",
                            "QuickMax",
                            "PreciseKp",
                            "PreciseMax",
                            "PreciseKd",
                            "PreciseMinPower",
                            "PreciseRateDeadband"
                    }[turretSel[0]];

                    linsane.telemetry().addData("Selected", sel);
                    linsane.telemetry().addData("QuickKp", "%6.4f", quickKp[0]);
                    linsane.telemetry().addData("QuickMax", "%5.2f", quickMax[0]);
                    linsane.telemetry().addData("PreciseKp", "%6.4f", preciseKp[0]);
                    linsane.telemetry().addData("PreciseMax", "%5.2f", preciseMax[0]);
                    linsane.telemetry().addData("PreciseKd", "%6.4f", preciseKd[0]);
                    linsane.telemetry().addData("PreciseMin", "%5.2f", preciseMin[0]);
                    linsane.telemetry().addData("PreciseRateDb", "%6.3f", preciseRateDb[0]);

                    linsane.telemetry().addData("Robot (X,Y,H)", "(%5.1f,%5.1f,%5.1f)", robotX, robotY, robotHeading);
                    linsane.telemetry().addData("GoalHeading", "%6.2f", goalHeading);
                    linsane.telemetry().addData("DesiredTurretDeg", "%6.2f", desiredTurretDeg);
                    linsane.telemetry().addData("TurretDeg", "%6.2f", turretDeg);
                    linsane.telemetry().addData("ErrDeg", "%6.2f", errDeg);

                    linsane.telemetry().addData("HasTag", usingTag);
                    linsane.telemetry().addLine();
                    linsane.telemetry().addData("YawDeg", "%6.2f", yawDeg);
                    linsane.telemetry().addData("CenterX(px)", Double.isNaN(centerX) ? "n/a" : String.format("%6.1f", centerX));
                    linsane.telemetry().addData("CenterErr(px)", Double.isNaN(errPx) ? "n/a" : String.format("%6.1f", errPx));
                    linsane.telemetry().addData("DegPerPx", Double.isNaN(degPerPx) ? "n/a" : String.format("%8.5f", degPerPx));

                    linsane.telemetry().addLine();
                    linsane.telemetry().addLine("Copy into Constants.Turret:");
                    linsane.telemetry().addData("QuickKp", "%6.4f", quickKp[0]);
                    linsane.telemetry().addData("QuickMaxPower", "%5.2f", quickMax[0]);
                    linsane.telemetry().addData("PreciseKp", "%6.4f", preciseKp[0]);
                    linsane.telemetry().addData("PreciseMaxPower", "%5.2f", preciseMax[0]);
                    linsane.telemetry().addData("PreciseKd", "%6.4f", preciseKd[0]);
                    linsane.telemetry().addData("PreciseMinPower", "%5.2f", preciseMin[0]);
                    linsane.telemetry().addData("PreciseRateDeadband", "%6.3f", preciseRateDb[0]);

                    linsane.telemetry().update();

                }))
        ));

        linsane.dropToScheduler();
    });
}