package org.firstinspires.ftc.teamcode.Util.Tuners;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.Constants;

@SuppressWarnings("unused")
@Configurable
public final class FlyWheelTuner {

    public static double targetXIn = 72.0;
    public static double targetYIn = 72.0;
    public static double driveThrottle = 1.0;
    public static double triggerThreshold = 0.10;

    private FlyWheelTuner() {}

    public static final Mercurial.RegisterableProgram flyWheelTuner = Mercurial.teleop(ctx -> {
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Intake.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Flywheel.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Turret.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Turret.INSTANCE.zeroTurret();

        ctx.schedule(sequence(
                waitUntil(ctx::inLoop),
                loop(exec(() -> {
                    double drive = -ctx.gamepad1().left_stick_y * driveThrottle;
                    double turn = ctx.gamepad1().right_stick_x * driveThrottle;

                    drive = Range.clip(drive, -1.0, 1.0);
                    turn = Range.clip(turn, -1.0, 1.0);

                    Drive.INSTANCE.updateOdometry();
                    Drive.INSTANCE.drive(drive, turn);

                    Turret.INSTANCE.lockTurret();

                    if (ctx.gamepad1().left_bumper) {
                        Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                    } else if (ctx.gamepad1().right_bumper) {
                        Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                    } else {
                        Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    }
                    Intake.INSTANCE.apply();

                    boolean charging = ctx.gamepad1().right_trigger > triggerThreshold;
                    if (charging) {
                        Flywheel.INSTANCE.setVelocityRps(Constants.Flywheel.TEST_RPS);
                    } else {
                        Flywheel.INSTANCE.stop();
                    }
                    Flywheel.INSTANCE.apply();

                    double x = Drive.INSTANCE.getX();
                    double y = Drive.INSTANCE.getY();
                    double heading = Drive.INSTANCE.getHeading();
                    double dx = targetXIn - x;
                    double dy = targetYIn - y;
                    double distance = Math.hypot(dx, dy);

                    ctx.telemetry().addLine("=== FlyWheel Tuner ===");
                    ctx.telemetry().addData("Flywheel1 RPS", "%5.2f", Flywheel.INSTANCE.getRps1());
                    ctx.telemetry().addData("Flywheel2 RPS", "%5.2f", Flywheel.INSTANCE.getRps2());
                    ctx.telemetry().addData("Avg RPS", "%5.2f", Flywheel.INSTANCE.getAverageRps());
                    ctx.telemetry().addData("Target RPS", Constants.Flywheel.TEST_RPS);
                    ctx.telemetry().addData("X", x);
                    ctx.telemetry().addData("Y", y);
                    ctx.telemetry().addData("Heading (deg)", heading);
                    ctx.telemetry().addData("Distance To Goal", Constants.Field.distanceToGoal());
                    ctx.telemetry().update();
                }))
        ));

        ctx.dropToScheduler();
    });
}
