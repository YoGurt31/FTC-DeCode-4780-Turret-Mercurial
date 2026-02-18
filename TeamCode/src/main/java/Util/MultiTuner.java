package Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import SubSystems.Drive;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Turret;
import SubSystems.Vision;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
public final class MultiTuner {

    private enum Mode { FLYWHEEL, TURRET, TRACKING }

    private static boolean rising(boolean now, boolean[] prev) {
        boolean r = now && !prev[0];
        prev[0] = now;
        return r;
    }

    private static double rpsToTicksPerSecond(double rps) {
        return rps * Constants.Flywheel.TICKS_PER_REV;
    }

    public static final Mercurial.RegisterableProgram TUNER = Mercurial.teleop(linsane -> {

        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        final Constants.Field.Alliance[] alliance = { Constants.Field.Alliance.RED };
        Vision.INSTANCE.setTrackedTag(alliance[0] == Constants.Field.Alliance.RED ? Constants.Vision.RED_TAG_ID : Constants.Vision.BLUE_TAG_ID);

        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        DcMotorEx fly1 = linsane.hardwareMap().get(DcMotorEx.class, Constants.Flywheel.Flywheel1);
        DcMotorEx fly2 = linsane.hardwareMap().get(DcMotorEx.class, Constants.Flywheel.Flywheel2);
        fly1.setDirection(DcMotorSimple.Direction.FORWARD);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        final Mode[] mode = { Mode.FLYWHEEL };

        final double[] steps = { 0.001, 0.0025, 0.005, 0.01, 0.02, 0.05, 0.1, 0.25, 0.5, 1.0 };
        final int[] stepIdx = { 3 };

        final long[] lastAdjustMs = { 0 };
        final long repeatMs = 120;

        final double[] flyP = { Constants.Flywheel.P };
        final double[] flyI = { Constants.Flywheel.I };
        final double[] flyD = { Constants.Flywheel.D };
        final double[] flyF = { Constants.Flywheel.F };
        final boolean[] flyOn = { false };
        final double[] flyTargetRps = { Constants.Flywheel.MIN_RPS };

        final int[] flySel = { 0 };

        final double[] trackGain = { Constants.Drive.ROTATE_GAIN };
        final double[] trackMaxRotate = { Constants.Drive.MAX_ROTATE };
        final int[] trackSel = { 0 };

        final double[] quickKp = { Constants.Turret.QuickKp };
        final double[] quickMax = { Constants.Turret.QuickMaxPower };
        final double[] preciseKp = { Constants.Turret.PreciseKp };
        final double[] preciseMax = { Constants.Turret.PreciseMaxPower };

        final int[] turretSel = { 0 };
        final boolean[] turretPrecise = { false };

        final boolean[] prevX = { false }, prevA = { false }, prevB = { false };
        final boolean[] prevDL = { false }, prevDR = { false }, prevLB = { false }, prevRB = { false };
        final boolean[] prevY = { false }, prevLS = { false };

        fly1.setVelocityPIDFCoefficients(flyP[0], flyI[0], flyD[0], flyF[0]);
        fly2.setVelocityPIDFCoefficients(flyP[0], flyI[0], flyD[0], flyF[0]);

        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                loop(exec(() -> {

                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();

                    if (rising(linsane.gamepad1().x, prevX)) mode[0] = Mode.FLYWHEEL;
                    if (rising(linsane.gamepad1().a, prevA)) mode[0] = Mode.TURRET;
                    if (rising(linsane.gamepad1().b, prevB)) mode[0] = Mode.TRACKING;

                    if (rising(linsane.gamepad1().dpad_left, prevDL)) stepIdx[0] = Math.max(0, stepIdx[0] - 1);
                    if (rising(linsane.gamepad1().dpad_right, prevDR)) stepIdx[0] = Math.min(steps.length - 1, stepIdx[0] + 1);
                    double step = steps[stepIdx[0]];

                    if (linsane.gamepad1().right_trigger > 0.05) {
                        Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                    } else if (linsane.gamepad1().left_trigger > 0.05) {
                        Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                    } else {
                        Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    }

                    double driveCmd = -linsane.gamepad1().left_stick_y;
                    double manualTurn = linsane.gamepad1().right_stick_x;

                    double turnCmd = manualTurn;
                    if (mode[0] == Mode.TRACKING) {
                        if (Vision.INSTANCE.hasArtifact()) {
                            double tx = Vision.INSTANCE.getTX();
                            double assist = Range.clip(tx * trackGain[0], -trackMaxRotate[0], trackMaxRotate[0]);
                            if (Math.abs(tx) <= Constants.Drive.ARTIFACT_AIM_DEADBAND_DEG) assist = 0.0;
                            turnCmd = assist;
                        } else {
                            turnCmd = manualTurn;
                        }
                    }
                    Drive.INSTANCE.drive(driveCmd, turnCmd);
                    Intake.INSTANCE.apply();

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

                    switch (mode[0]) {

                        case FLYWHEEL: {
                            if (rising(linsane.gamepad1().left_bumper, prevLB)) flySel[0] = (flySel[0] + 2) % 3;
                            if (rising(linsane.gamepad1().right_bumper, prevRB)) flySel[0] = (flySel[0] + 1) % 3;

                            if (rising(linsane.gamepad1().y, prevY)) flyOn[0] = !flyOn[0];

                            if (doAdjust) {
                                if (flySel[0] == 0) flyP[0] = Math.max(0.0, flyP[0] + dir * step);
                                else if (flySel[0] == 1) flyF[0] = Math.max(0.0, flyF[0] + dir * step);
                                else flyTargetRps[0] = Range.clip(flyTargetRps[0] + dir * step, Constants.Flywheel.MIN_RPS, Constants.Flywheel.MAX_RPS);

                                fly1.setVelocityPIDFCoefficients(flyP[0], flyI[0], flyD[0], flyF[0]);
                                fly2.setVelocityPIDFCoefficients(flyP[0], flyI[0], flyD[0], flyF[0]);
                            }

                            if (flyOn[0]) {
                                double tps = rpsToTicksPerSecond(flyTargetRps[0]);
                                fly1.setVelocity(tps);
                                fly2.setVelocity(tps);
                            } else {
                                fly1.setPower(0.0);
                                fly2.setPower(0.0);
                            }

                            double rps1 = fly1.getVelocity() / Constants.Flywheel.TICKS_PER_REV;
                            double rps2 = fly2.getVelocity() / Constants.Flywheel.TICKS_PER_REV;
                            double avg = (rps1 + rps2) / 2.0;

                            linsane.telemetry().addLine("=== Flywheel PIDF Tuner (X) ===");
                            linsane.telemetry().addLine("Y toggle ON/OFF | LB/RB cycle param | Dpad L/R step | Dpad U/D adjust");
                            linsane.telemetry().addData("Step", step);
                            linsane.telemetry().addData("Selected", (flySel[0] == 0) ? "P" : (flySel[0] == 1) ? "F" : "TargetRPS");
                            linsane.telemetry().addData("ON", flyOn[0]);
                            linsane.telemetry().addData("TargetRPS", "%5.2f", flyTargetRps[0]);
                            linsane.telemetry().addData("RPS1", "%5.2f", rps1);
                            linsane.telemetry().addData("RPS2", "%5.2f", rps2);
                            linsane.telemetry().addData("Avg", "%5.2f", avg);
                            linsane.telemetry().addData("P", "%6.4f", flyP[0]);
                            linsane.telemetry().addData("F", "%6.4f", flyF[0]);
                            linsane.telemetry().addLine();
                            linsane.telemetry().addLine("Copy into Constants.Flywheel & Send to Gurt:");
                            linsane.telemetry().addData("P", "%6.4f", flyP[0]);
                            linsane.telemetry().addData("F", "%6.4f", flyF[0]);
                            linsane.telemetry().addLine();
                            linsane.telemetry().addLine("Dynamic RPS Data:");
                            linsane.telemetry().addData("RPS", "%5.2f", avg);
                            linsane.telemetry().addData("Robot (X,Y)", "(%5.1f,%5.1f)", Drive.INSTANCE.getX(), Drive.INSTANCE.getY());
                            break;
                        }

                        case TURRET: {
                            int tagId = (alliance[0] == Constants.Field.Alliance.RED)
                                    ? Constants.Vision.RED_TAG_ID
                                    : Constants.Vision.BLUE_TAG_ID;
                            Vision.INSTANCE.setTrackedTag(tagId);
                            if (rising(linsane.gamepad1().y, prevY)) turretPrecise[0] = !turretPrecise[0];

                            if (rising(linsane.gamepad1().left_stick_button, prevLS)) {
                                alliance[0] = (alliance[0] == Constants.Field.Alliance.RED)
                                        ? Constants.Field.Alliance.BLUE
                                        : Constants.Field.Alliance.RED;
                                tagId = (alliance[0] == Constants.Field.Alliance.RED)
                                        ? Constants.Vision.RED_TAG_ID
                                        : Constants.Vision.BLUE_TAG_ID;
                                Vision.INSTANCE.setTrackedTag(tagId);
                            }

                            if (rising(linsane.gamepad1().left_bumper, prevLB)) turretSel[0] = (turretSel[0] + 3) % 4;
                            if (rising(linsane.gamepad1().right_bumper, prevRB)) turretSel[0] = (turretSel[0] + 1) % 4;

                            if (doAdjust) {
                                switch (turretSel[0]) {
                                    case 0: quickKp[0] = Math.max(0.0, quickKp[0] + dir * step); break;
                                    case 1: quickMax[0] = Range.clip(quickMax[0] + dir * step, 0.0, 1.0); break;
                                    case 2: preciseKp[0] = Math.max(0.0, preciseKp[0] + dir * step); break;
                                    case 3: preciseMax[0] = Range.clip(preciseMax[0] + dir * step, 0.0, 1.0); break;
                                }
                            }

                            double robotX = Drive.INSTANCE.getX();
                            double robotY = Drive.INSTANCE.getY();
                            double robotHeading = Drive.INSTANCE.getHeading();
                            double goalHeading = Constants.Field.computeGoalHeadingDeg(robotX, robotY, alliance[0]);
                            boolean atTarget = Turret.INSTANCE.autoAimTurretTunable(robotHeading, goalHeading, quickKp[0], quickMax[0], preciseKp[0], preciseMax[0], turretPrecise[0]);

                            double turretDeg = Turret.INSTANCE.getTurretDeg();
                            double desiredTurretDeg = Turret.INSTANCE.getTargetDeg();
                            double errDeg = Turret.INSTANCE.getErrorDeg();

                            boolean usingTag = turretPrecise[0] && Vision.INSTANCE.hasTrackedTag();
                            double yawDeg = usingTag ? Vision.INSTANCE.getTrackedYawDeg() : 0.0;

                            linsane.telemetry().addLine("=== Turret Tuner (A) ===");
                            linsane.telemetry().addLine("Y toggle Quick/Precise | L3 click toggle Alliance | LB/RB cycle param | Dpad L/R step | Dpad U/D adjust");
                            linsane.telemetry().addData("Alliance", alliance[0]);
                            linsane.telemetry().addData("TagId", (alliance[0] == Constants.Field.Alliance.RED) ? Constants.Vision.RED_TAG_ID : Constants.Vision.BLUE_TAG_ID);
                            linsane.telemetry().addData("Mode", turretPrecise[0] ? "PRECISE (Cam)" : "QUICK (PinPoint)");
                            linsane.telemetry().addData("AtTarget", atTarget);
                            linsane.telemetry().addData("Step", step);
                            String sel = new String[]{"QuickKp","QuickMax","PreciseKp","PreciseMax"}[turretSel[0]];
                            linsane.telemetry().addData("Selected", sel);
                            linsane.telemetry().addData("QuickKp", "%6.4f", quickKp[0]);
                            linsane.telemetry().addData("QuickMax", "%5.2f", quickMax[0]);
                            linsane.telemetry().addData("PreciseKp", "%6.4f", preciseKp[0]);
                            linsane.telemetry().addData("PreciseMax", "%5.2f", preciseMax[0]);
                            linsane.telemetry().addData("Robot (X,Y,H)", "(%5.1f,%5.1f,%5.1f)", robotX, robotY, robotHeading);
                            linsane.telemetry().addData("GoalHeading", "%6.2f", goalHeading);
                            linsane.telemetry().addData("DesiredTurretDeg", "%6.2f", desiredTurretDeg);
                            linsane.telemetry().addData("TurretDeg", "%6.2f", turretDeg);
                            linsane.telemetry().addData("ErrDeg", "%6.2f", errDeg);
                            linsane.telemetry().addData("HasTag", usingTag);
                            linsane.telemetry().addData("YawDeg", "%6.2f", yawDeg);
                            linsane.telemetry().addLine();
                            linsane.telemetry().addLine("Copy into Constants.Turret & Send to Gurt:");
                            linsane.telemetry().addData("QuickKp", "%6.4f", quickKp[0]);
                            linsane.telemetry().addData("QuickMax", "%5.2f", quickMax[0]);
                            linsane.telemetry().addData("PreciseKp", "%6.4f", preciseKp[0]);
                            linsane.telemetry().addData("PreciseMax", "%5.2f", preciseMax[0]);
                            break;
                        }

                        case TRACKING: {
                            if (rising(linsane.gamepad1().left_bumper, prevLB)) trackSel[0] = (trackSel[0] + 1) % 2;
                            if (rising(linsane.gamepad1().right_bumper, prevRB)) trackSel[0] = (trackSel[0] + 1) % 2;

                            if (doAdjust) {
                                if (trackSel[0] == 0) trackGain[0] = Math.max(0.0, trackGain[0] + dir * step);
                                else trackMaxRotate[0] = Range.clip(trackMaxRotate[0] + dir * step, 0.0, 1.0);
                            }

                            linsane.telemetry().addLine("=== Artifact Tracking Tuner (B) ===");
                            linsane.telemetry().addLine("Bring Artifact to Intake in view of LL. Robot should auto rotate to center artifact.");
                            linsane.telemetry().addLine("LB/RB cycle param | Dpad L/R step | Dpad U/D adjust");
                            linsane.telemetry().addData("Step", step);
                            linsane.telemetry().addData("Selected", (trackSel[0] == 0) ? "ROTATE_GAIN" : "MAX_ROTATE");
                            linsane.telemetry().addData("ROTATE_GAIN", "%6.4f", trackGain[0]);
                            linsane.telemetry().addData("MAX_ROTATE", "%5.2f", trackMaxRotate[0]);
                            linsane.telemetry().addData("HasArtifact", Vision.INSTANCE.hasArtifact());
                            linsane.telemetry().addData("TX", "%6.2f", Vision.INSTANCE.getTX());
                            linsane.telemetry().addLine();
                            linsane.telemetry().addLine("Copy into Constants.Drive & Send to Gurt:");
                            linsane.telemetry().addData("ROTATE_GAIN", "%6.4f", trackGain[0]);
                            linsane.telemetry().addData("MAX_ROTATE", "%5.2f", trackMaxRotate[0]);
                            break;
                        }
                    }

                    linsane.telemetry().addData("Active Mode", mode[0]);
                    linsane.telemetry().update();

                }))
        ));

        linsane.dropToScheduler();
    });
}