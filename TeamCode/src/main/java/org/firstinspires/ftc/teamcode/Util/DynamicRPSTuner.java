package org.firstinspires.ftc.teamcode.Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Release;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@SuppressWarnings("unused")
public final class DynamicRPSTuner {

    private static final double CHARGE_TRIGGER_THRESHOLD = 0.10;
    private static final double DRIVE_DEADBAND = 0.05;

    private static Mercurial.RegisterableProgram buildTeleOp(Constants.Field.Alliance alliance, boolean telemetry) {
        return Mercurial.teleop(ctx -> {

            Drive.INSTANCE.setResetPinPointOnInit(true);
            Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

            Constants.Field.setAlliance(alliance);

            Intake.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
            Turret.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
            Flywheel.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
            Release.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

            final double[] targetRps = {Flywheel.INSTANCE.getTargetRps()};

            ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_up), exec(() -> targetRps[0] += 1.0));
            ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_down), exec(() -> targetRps[0] -= 1.0));
            ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_right), exec(() -> targetRps[0] += 0.5));
            ctx.bindSpawn(ctx.risingEdge(() -> ctx.gamepad1().dpad_left), exec(() -> targetRps[0] -= 0.5));

            ctx.schedule(sequence(waitUntil(ctx::inLoop), loop(exec(() -> {

                Drive.INSTANCE.updateOdometry();

                double driveCmd = -ctx.gamepad1().left_stick_y;
                double turnCmd = ctx.gamepad1().right_stick_x;

                if (Math.abs(driveCmd) < DRIVE_DEADBAND) driveCmd = 0.0;
                if (Math.abs(turnCmd) < DRIVE_DEADBAND) turnCmd = 0.0;

                if (ctx.gamepad1().left_bumper) {
                    Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                } else if (ctx.gamepad1().right_bumper) {
                    Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                } else {
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                }

                Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), Constants.Field.computeGoalHeadingDeg(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), alliance));

                targetRps[0] = Math.max(0.0, targetRps[0]);

                boolean charging = ctx.gamepad1().right_trigger >= CHARGE_TRIGGER_THRESHOLD;

                if (charging) {
                    Flywheel.INSTANCE.setVelocityRps(targetRps[0]);
                    Release.INSTANCE.open();
                } else {
                    Flywheel.INSTANCE.setVelocityRps(Constants.Flywheel.MIN_RPS);
                    Release.INSTANCE.close();
                }

                Drive.INSTANCE.drive(driveCmd, turnCmd);
                Intake.INSTANCE.apply();
                Flywheel.INSTANCE.apply();
                Release.INSTANCE.apply();

                if (telemetry) {
                    ctx.telemetry().addData("Alliance", alliance);
                    ctx.telemetry().addData("Current RPS", String.format("%.1f", Flywheel.INSTANCE.getAverageRps()));
                    ctx.telemetry().addData("Target RPS", String.format("%.1f", targetRps[0]));
                    ctx.telemetry().addData("Charging", charging);
                    ctx.telemetry().addData("Pose (in)", String.format("x=%.1f y=%.1f h=%.1f", Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), Drive.INSTANCE.getHeading()));
                    ctx.telemetry().addData("Distance (in)", String.format("%.1f", Constants.Field.distanceToGoal()));
                    ctx.telemetry().update();
                }

            }))));

            ctx.dropToScheduler();
        });
    }

    public static final Mercurial.RegisterableProgram DynamicRPSTuner = buildTeleOp(Constants.Field.Alliance.BLUE, true);
}