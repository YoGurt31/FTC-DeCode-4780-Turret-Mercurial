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
import Util.Constants;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
public final class FlyWheelTuner {

    private static double rpsToTicksPerSecond(double rps) {
        return rps * Constants.Flywheel.TICKS_PER_REV;
    }

    public static final Mercurial.RegisterableProgram DynamicRPS = Mercurial.teleop(linsane -> {

        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        final Constants.Field.Alliance[] alliance = { Constants.Field.Alliance.BLUE };
        Constants.Field.setAlliance(alliance[0]);
        Vision.INSTANCE.setTrackedTag(alliance[0] == Constants.Field.Alliance.RED ? Constants.Vision.RED_TAG_ID : Constants.Vision.BLUE_TAG_ID);

        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        final long[] lastAdjustMs = { 0 };
        final long repeatMs = 120;

        final double[] flyTargetRps = { Constants.Flywheel.MIN_RPS };

        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                loop(exec(() -> {

                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();

                    double goalHeading = Constants.Field.computeGoalHeadingDeg(
                            Drive.INSTANCE.getX(),
                            Drive.INSTANCE.getY(),
                            alliance[0]
                    );
                    Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), goalHeading);

                    long now = System.currentTimeMillis();
                    boolean doAdjust = false;
                    int dir = 0;
                    double step = 0.0;

                    if (linsane.gamepad1().dpad_up) {
                        dir = +1;
                        step = 1.0;
                        doAdjust = (now - lastAdjustMs[0]) >= repeatMs;
                    } else if (linsane.gamepad1().dpad_down) {
                        dir = -1;
                        step = 1.0;
                        doAdjust = (now - lastAdjustMs[0]) >= repeatMs;
                    } else if (linsane.gamepad1().dpad_right) {
                        dir = +1;
                        step = 0.1;
                        doAdjust = (now - lastAdjustMs[0]) >= repeatMs;
                    } else if (linsane.gamepad1().dpad_left) {
                        dir = -1;
                        step = 0.1;
                        doAdjust = (now - lastAdjustMs[0]) >= repeatMs;
                    } else {
                        lastAdjustMs[0] = 0;
                    }

                    if (doAdjust) {
                        lastAdjustMs[0] = now;
                        flyTargetRps[0] = Range.clip(
                                flyTargetRps[0] + dir * step,
                                Constants.Flywheel.MIN_RPS,
                                Constants.Flywheel.MAX_RPS
                        );
                    }

                    if (linsane.gamepad1().left_bumper) {
                        Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                    } else if (linsane.gamepad1().right_bumper) {
                        Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                    } else {
                        Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    }
                    Intake.INSTANCE.setScale(Constants.Field.inFarZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY()) ? Constants.Intake.TRANSFER_SCALE_FAR : Constants.Intake.TRANSFER_SCALE_CLOSE);

                    if (linsane.gamepad1().right_trigger > 0.05) {
                        Flywheel.INSTANCE.setVelocityRps(flyTargetRps[0]);
                    } else {
                        Flywheel.INSTANCE.stop();
                    }

                    double driveCmd = -linsane.gamepad1().left_stick_y;
                    double turnCmd = linsane.gamepad1().right_stick_x;

                    Drive.INSTANCE.drive(driveCmd, turnCmd);
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();

                    linsane.telemetry().addLine("=== Flywheel RPS Tuner + Turret Auto-Aim ===");
                    linsane.telemetry().addLine("Dpad U/D = +/-1.0 RPS | Dpad L/R = +/-0.1 RPS");
                    linsane.telemetry().addData("Step", step);
                    linsane.telemetry().addData("ON", linsane.gamepad1().right_trigger > 0.05);
                    linsane.telemetry().addData("TargetRPS", "%5.2f", flyTargetRps[0]);
                    linsane.telemetry().addData("Flywheel1 RPS", "%5.2f", Flywheel.INSTANCE.getRps1());
                    linsane.telemetry().addData("Flywheel2 RPS", "%5.2f", Flywheel.INSTANCE.getRps2());
                    linsane.telemetry().addData("Avg RPS", "%5.2f", Flywheel.INSTANCE.getAverageRps());
                    linsane.telemetry().addData("TurretDeg", "%6.2f", Turret.INSTANCE.getTurretDeg());
                    linsane.telemetry().addData("GoalHeading", "%6.2f", goalHeading);
                    linsane.telemetry().addLine();
                    linsane.telemetry().addLine("Dynamic RPS Data:");
                    linsane.telemetry().addData("RPS", "%5.2f", Flywheel.INSTANCE.getAverageRps());
                    linsane.telemetry().addData("Robot (X,Y)", "(%5.1f,%5.1f)", Drive.INSTANCE.getX(), Drive.INSTANCE.getY());
                    linsane.telemetry().update();

                }))
        ));

        linsane.dropToScheduler();
    });
}