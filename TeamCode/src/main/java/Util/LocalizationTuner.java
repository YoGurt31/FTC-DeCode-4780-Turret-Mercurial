package Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import SubSystems.Drive;
import SubSystems.Turret;
import SubSystems.Vision;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
public final class LocalizationTuner {

    private static boolean rising(boolean now, boolean[] prev) {
        boolean r = now && !prev[0];
        prev[0] = now;
        return r;
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    private static final class Conv {
        double xIn;
        double yIn;
        double yawDeg;
        boolean ok;
    }

    private static Conv convertLL(Pose3D llPose, boolean swapXY, double xSign, double ySign, double yawSign, double yawOffsetDeg, double metersToIn) {
        Conv out = new Conv();
        out.ok = false;
        if (llPose == null) return out;

        double xIn = llPose.getPosition().x * metersToIn;
        double yIn = llPose.getPosition().y * metersToIn;

        if (swapXY) {
            double t = xIn;
            xIn = yIn;
            yIn = t;
        }

        xIn *= xSign;
        yIn *= ySign;

        double yawDeg = llPose.getOrientation().getYaw(AngleUnit.DEGREES);
        yawDeg = wrapDeg(yawDeg * yawSign + yawOffsetDeg);

        out.xIn = xIn;
        out.yIn = yIn;
        out.yawDeg = yawDeg;
        out.ok = true;
        return out;
    }

    public static final Mercurial.RegisterableProgram Localizer = Mercurial.teleop(linsane -> {

        Drive.INSTANCE.setResetPinPointOnInit(false);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        final Constants.Field.Alliance[] alliance = {Constants.Field.Alliance.BLUE};
        Constants.Field.setAlliance(alliance[0]);

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Vision.INSTANCE.setPipeline(Constants.Vision.LOCALIZATION_PIPELINE);

        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        final boolean[] swapXY = {Constants.Relocalize.SWAP_XY};
        final double[] xSign = {(double) Constants.Relocalize.LL_X_TO_PP_SIGN};
        final double[] ySign = {(double) Constants.Relocalize.LL_Y_TO_PP_SIGN};
        final double[] yawSign = {Constants.Relocalize.LL_YAW_SIGN};
        final double[] yawOffset = {Constants.Relocalize.LL_YAW_OFFSET_DEG};

        final double metersToIn = Constants.Relocalize.METERS_TO_IN;

        final double[] maxDistJump = {Constants.Relocalize.MAX_DIST_JUMP_IN};
        final double[] maxYawJump = {Constants.Relocalize.MAX_YAW_JUMP_DEG};

        final boolean[] autoApply = {false};
        final long[] lastApplyMs = {0};

        final double stickDb = Constants.Relocalize.STATIONARY_STICK_DB;
        final long stationaryMs = Constants.Relocalize.STATIONARY_TIME_MS;
        final long cooldownMs = Constants.Relocalize.COOLDOWN_MS;
        final long[] stationarySince = {0};

        final double[] steps = {0.25, 0.5, 1.0, 2.5, 5.0};
        final int[] stepIdx = {2};

        final int[] sel = {0};

        final boolean[] prevDL = {false}, prevDR = {false}, prevLB = {false}, prevRB = {false};
        final boolean[] prevA = {false}, prevY = {false}, prevX = {false}, prevB = {false}, prevLS = {false};

        linsane.schedule(sequence(waitUntil(linsane::inLoop), loop(exec(() -> {

            Drive.INSTANCE.updateOdometry();
            Vision.INSTANCE.updateRobotYawDeg(Drive.INSTANCE.getHeading());
            Vision.INSTANCE.update();

            Vision.INSTANCE.setPipeline(Constants.Vision.LOCALIZATION_PIPELINE);

            if (rising(linsane.gamepad1().left_stick_button, prevLS)) {
                alliance[0] = (alliance[0] == Constants.Field.Alliance.BLUE) ? Constants.Field.Alliance.RED : Constants.Field.Alliance.BLUE;
                Constants.Field.setAlliance(alliance[0]);
            }

            double driveCmd = -linsane.gamepad1().left_stick_y;
            double turnCmd = linsane.gamepad1().right_stick_x;
            Drive.INSTANCE.drive(driveCmd, turnCmd);

            double goalHeading = Constants.Field.computeGoalHeadingDeg(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), alliance[0]);
            Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), goalHeading);

            long now = System.currentTimeMillis();
            boolean stationary = Math.abs(driveCmd) < stickDb && Math.abs(turnCmd) < stickDb;
            if (stationary) {
                if (stationarySince[0] == 0) stationarySince[0] = now;
            } else {
                stationarySince[0] = 0;
            }
            boolean stationaryLongEnough = stationarySince[0] != 0 && (now - stationarySince[0]) >= stationaryMs;
            boolean cooldownOk = (now - lastApplyMs[0]) >= cooldownMs;

            Pose3D ll = Vision.INSTANCE.getPose();
            Conv c = convertLL(ll, swapXY[0], xSign[0], ySign[0], yawSign[0], yawOffset[0], metersToIn);

            double ppX = Drive.INSTANCE.getX();
            double ppY = Drive.INSTANCE.getY();
            double ppH = Drive.INSTANCE.getHeading();

            double dx = c.ok ? (c.xIn - ppX) : Double.NaN;
            double dy = c.ok ? (c.yIn - ppY) : Double.NaN;
            double dh = c.ok ? wrapDeg(c.yawDeg - ppH) : Double.NaN;

            boolean jumpOk = c.ok && (Math.hypot(dx, dy) <= maxDistJump[0]) && (Math.abs(dh) <= maxYawJump[0]);

            if (rising(linsane.gamepad1().dpad_left, prevDL))
                stepIdx[0] = Math.max(0, stepIdx[0] - 1);
            if (rising(linsane.gamepad1().dpad_right, prevDR))
                stepIdx[0] = Math.min(steps.length - 1, stepIdx[0] + 1);
            double step = steps[stepIdx[0]];

            if (rising(linsane.gamepad1().left_bumper, prevLB)) sel[0] = (sel[0] + 6) % 7;
            if (rising(linsane.gamepad1().right_bumper, prevRB)) sel[0] = (sel[0] + 1) % 7;

            if (rising(linsane.gamepad1().y, prevY)) autoApply[0] = !autoApply[0];

            if (rising(linsane.gamepad1().x, prevX)) {
                swapXY[0] = Constants.Relocalize.SWAP_XY;
                xSign[0] = (double) Constants.Relocalize.LL_X_TO_PP_SIGN;
                ySign[0] = (double) Constants.Relocalize.LL_Y_TO_PP_SIGN;
                yawSign[0] = Constants.Relocalize.LL_YAW_SIGN;
                yawOffset[0] = Constants.Relocalize.LL_YAW_OFFSET_DEG;
                maxDistJump[0] = Constants.Relocalize.MAX_DIST_JUMP_IN;
                maxYawJump[0] = Constants.Relocalize.MAX_YAW_JUMP_DEG;
            }

            if (rising(linsane.gamepad1().b, prevB)) yawOffset[0] = 0.0;

            if (linsane.gamepad1().dpad_up || linsane.gamepad1().dpad_down) {
                double dir = linsane.gamepad1().dpad_up ? +1.0 : -1.0;
                switch (sel[0]) {
                    case 4:
                        yawOffset[0] = wrapDeg(yawOffset[0] + dir * step);
                        break;
                    case 5:
                        maxDistJump[0] = Math.max(0.0, maxDistJump[0] + dir * step);
                        break;
                    case 6:
                        maxYawJump[0] = Math.max(0.0, maxYawJump[0] + dir * step);
                        break;
                }
            }

            if (rising(linsane.gamepad1().a, prevA)) {
                if (sel[0] == 0) {
                    swapXY[0] = !swapXY[0];
                } else if (sel[0] == 1) {
                    xSign[0] = (xSign[0] >= 0) ? -1.0 : +1.0;
                } else if (sel[0] == 2) {
                    ySign[0] = (ySign[0] >= 0) ? -1.0 : +1.0;
                } else if (sel[0] == 3) {
                    yawSign[0] = (yawSign[0] >= 0) ? -1.0 : +1.0;
                } else {
                    if (c.ok && jumpOk && stationaryLongEnough && cooldownOk) {
                        Drive.INSTANCE.setPose(c.xIn, c.yIn, c.yawDeg);
                        lastApplyMs[0] = now;
                        stationarySince[0] = now;
                    }
                }
            }

            if (autoApply[0] && c.ok && jumpOk && stationaryLongEnough && cooldownOk) {
                Drive.INSTANCE.setPose(c.xIn, c.yIn, c.yawDeg);
                lastApplyMs[0] = now;
                stationarySince[0] = now;
            }

            linsane.telemetry().addLine("=== Localization Tuner (Limelight -> PinPoint) ===");
            linsane.telemetry().addLine("Drive: LS Y / RS X | Turret auto-aim ON");
            linsane.telemetry().addLine("L3 alliance | LB/RB select | Dpad L/R step | Dpad U/D adjust (numeric) | A toggle/apply | Y auto | X reset | B yawOffset=0");

            String[] names = {"SWAP_XY", "LL_X_TO_PP_SIGN", "LL_Y_TO_PP_SIGN", "LL_YAW_SIGN", "LL_YAW_OFFSET_DEG", "MAX_DIST_JUMP_IN", "MAX_YAW_JUMP_DEG"};
            linsane.telemetry().addData("Alliance", alliance[0]);
            linsane.telemetry().addData("Selected", names[sel[0]]);
            linsane.telemetry().addData("Step", step);

            linsane.telemetry().addData("SWAP_XY", swapXY[0]);
            linsane.telemetry().addData("X Sign", xSign[0]);
            linsane.telemetry().addData("Y Sign", ySign[0]);
            linsane.telemetry().addData("Yaw Sign", yawSign[0]);
            linsane.telemetry().addData("Yaw Offset", "%6.2f", yawOffset[0]);
            linsane.telemetry().addData("MaxDistJump", "%6.2f", maxDistJump[0]);
            linsane.telemetry().addData("MaxYawJump", "%6.2f", maxYawJump[0]);
            linsane.telemetry().addData("AutoApply", autoApply[0]);

            linsane.telemetry().addLine();
            linsane.telemetry().addData("PinPoint (X,Y,H)", "(%6.1f, %6.1f, %6.1f)", ppX, ppY, ppH);

            if (ll == null) {
                linsane.telemetry().addData("Limelight Pose", "none / invalid");
            } else {
                linsane.telemetry().addData("LL raw (m)", "(%6.3f, %6.3f)", ll.getPosition().x, ll.getPosition().y);
                linsane.telemetry().addData("LL yaw(deg)", "%6.1f", ll.getOrientation().getYaw(AngleUnit.DEGREES));
            }

            if (c.ok) {
                linsane.telemetry().addData("LL->PP (X,Y,H)", "(%6.1f, %6.1f, %6.1f)", c.xIn, c.yIn, c.yawDeg);
                linsane.telemetry().addData("Delta (dX,dY,dH)", "(%6.1f, %6.1f, %6.1f)", dx, dy, dh);
                linsane.telemetry().addData("JumpOK", jumpOk);
            }

            linsane.telemetry().addLine();
            linsane.telemetry().addData("Stationary", stationary);
            linsane.telemetry().addData("StationaryLongEnough", stationaryLongEnough);
            linsane.telemetry().addData("CooldownOK", cooldownOk);
            linsane.telemetry().addData("Pipeline", Vision.INSTANCE.getPipelineIndex());

            linsane.telemetry().addLine();
            linsane.telemetry().addLine("Copy into Constants.Relocalize:");
            linsane.telemetry().addData("SWAP_XY", swapXY[0]);
            linsane.telemetry().addData("LL_X_TO_PP_SIGN", (int) xSign[0]);
            linsane.telemetry().addData("LL_Y_TO_PP_SIGN", (int) ySign[0]);
            linsane.telemetry().addData("LL_YAW_SIGN", yawSign[0]);
            linsane.telemetry().addData("LL_YAW_OFFSET_DEG", "%6.2f", yawOffset[0]);
            linsane.telemetry().addData("MAX_DIST_JUMP_IN", "%6.2f", maxDistJump[0]);
            linsane.telemetry().addData("MAX_YAW_JUMP_DEG", "%6.2f", maxYawJump[0]);

            linsane.telemetry().update();

        }))));

        linsane.dropToScheduler();
    });
}