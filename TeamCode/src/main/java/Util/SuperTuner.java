package Util;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import SubSystems.Drive;
import SubSystems.Flywheel;
import SubSystems.Turret;
import SubSystems.Vision;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
@Configurable
public final class SuperTuner {

    private enum Mode { TURRET, FLYWHEEL, TRACKING }

    public static final Mercurial.RegisterableProgram TUNER = Mercurial.teleop(linsane -> {

        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

        final Mode[] mode = { Mode.TURRET };
        final Util.Constants.Field.Alliance[] alliance = { Util.Constants.Field.Alliance.BLUE };

        final boolean[] prevL1 = { false };
        final boolean[] prevR1 = { false };
        final boolean[] prevA  = { false };
        final boolean[] prevB  = { false };

        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                loop(exec(() -> {

                    boolean l1 = linsane.gamepad1().left_bumper;
                    boolean r1 = linsane.gamepad1().right_bumper;

                    boolean l1Rise = l1 && !prevL1[0];
                    boolean r1Rise = r1 && !prevR1[0];
                    prevL1[0] = l1;
                    prevR1[0] = r1;

                    if (l1Rise) {
                        mode[0] = (mode[0] == Mode.TURRET) ? Mode.TRACKING
                                : (mode[0] == Mode.FLYWHEEL) ? Mode.TURRET
                                : Mode.FLYWHEEL;
                    }
                    if (r1Rise) {
                        mode[0] = (mode[0] == Mode.TURRET) ? Mode.FLYWHEEL
                                : (mode[0] == Mode.FLYWHEEL) ? Mode.TRACKING
                                : Mode.TURRET;
                    }

                    Drive.INSTANCE.updateOdometry();
                    Vision.INSTANCE.update();

                    boolean a = linsane.gamepad1().a;
                    boolean b = linsane.gamepad1().b;
                    boolean aRise = a && !prevA[0];
                    boolean bRise = b && !prevB[0];
                    prevA[0] = a;
                    prevB[0] = b;

                    if (mode[0] == Mode.TURRET) {
                        if (aRise) alliance[0] = Util.Constants.Field.Alliance.BLUE;
                        if (bRise) alliance[0] = Util.Constants.Field.Alliance.RED;
                        Util.Constants.Field.setAlliance(alliance[0]);
                    }

                    switch (mode[0]) {

                        case TURRET: {
                            double driveCmd = -linsane.gamepad1().left_stick_y;
                            double turnCmd  =  linsane.gamepad1().right_stick_x;
                            Drive.INSTANCE.drive(driveCmd, turnCmd);

                            double x = Drive.INSTANCE.getX();
                            double y = Drive.INSTANCE.getY();
                            double h = Drive.INSTANCE.getHeading();
                            double goalHeading = Util.Constants.Field.computeGoalHeadingDeg(x, y, alliance[0]);

                            Turret.INSTANCE.autoAimTurretTunable(
                                    h, goalHeading,
                                    Util.Constants.Tunable.TurKp,
                                    Util.Constants.Tunable.TurKd,
                                    Util.Constants.Tunable.TurMin,
                                    Util.Constants.Tunable.TurMax
                            );

                            Flywheel.INSTANCE.stop();
                            break;
                        }

                        case FLYWHEEL: {
                            double driveCmd = -linsane.gamepad1().left_stick_y;
                            double turnCmd  =  linsane.gamepad1().right_stick_x;
                            Drive.INSTANCE.drive(driveCmd, turnCmd);

                            if (linsane.gamepad1().right_trigger > 0.1) {
                                Flywheel.INSTANCE.setVelocityRps(Util.Constants.Tunable.FLYWHEEL_TARGET_RPS);
                                Flywheel.INSTANCE.apply();
                            } else {
                                Flywheel.INSTANCE.stop();
                            }

                            break;
                        }

                        case TRACKING: {
                            Vision.INSTANCE.setPipeline(Util.Constants.Vision.ARTIFACT_PIPELINE);

                            double turn = 0.0;
                            if (Vision.INSTANCE.hasArtifact()) {
                                double tx = Vision.INSTANCE.getTX();
                                if (Math.abs(tx) > Util.Constants.Tunable.DRIVE_ARTIFACT_AIM_DEADBAND_DEG) {
                                    turn = Range.clip(tx * Util.Constants.Tunable.DRIVE_ROTATE_GAIN,
                                            -Util.Constants.Tunable.DRIVE_MAX_ROTATE,
                                            +Util.Constants.Tunable.DRIVE_MAX_ROTATE);
                                }
                            }
                            Drive.INSTANCE.drive(0.0, turn);

                            Flywheel.INSTANCE.stop();
                            break;
                        }
                    }

                    linsane.telemetry().addData("Mode", mode[0]);
                    linsane.telemetry().addData("Alliance", alliance[0]);
                    linsane.telemetry().addData("Pose (x,y,h)", "(%.1f, %.1f, %.1f)",
                            Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), Drive.INSTANCE.getHeading());
                    linsane.telemetry().addLine();

                    if (mode[0] == Mode.TURRET) {
                        double x = Drive.INSTANCE.getX();
                        double y = Drive.INSTANCE.getY();
                        double h = Drive.INSTANCE.getHeading();
                        double goalHeading = Util.Constants.Field.computeGoalHeadingDeg(x, y, alliance[0]);

                        linsane.telemetry().addLine("--- Turret ---");
                        linsane.telemetry().addData("Robot Heading", "%.1f deg", h);
                        linsane.telemetry().addData("Goal Heading", "%.1f deg", goalHeading);
                        linsane.telemetry().addData("Turret Heading", "%.1f deg", Turret.INSTANCE.getTurretDeg());
                        linsane.telemetry().addData("kP", Util.Constants.Tunable.TurKp);
                        linsane.telemetry().addData("kD", Util.Constants.Tunable.TurKd);
                        linsane.telemetry().addData("Min Power", Util.Constants.Tunable.TurMin);
                        linsane.telemetry().addData("Max Power", Util.Constants.Tunable.TurMax);
                    }
                    else if (mode[0] == Mode.FLYWHEEL) {
                        double rt = linsane.gamepad1().right_trigger;

                        linsane.telemetry().addLine("--- Flywheel ---");
                        linsane.telemetry().addData("RT", "%.2f", rt);
                        linsane.telemetry().addData("Target RPS", Util.Constants.Tunable.FLYWHEEL_TARGET_RPS);
                        linsane.telemetry().addData("RPS1", "%.1f", Flywheel.INSTANCE.getRps1(), "RPS2", "%.1f", Flywheel.INSTANCE.getRps2());
                        linsane.telemetry().addData("Avg RPS", "%.1f", Flywheel.INSTANCE.getAverageRps());
                        linsane.telemetry().addData("P", Util.Constants.Tunable.FLYWHEEL_P);
                        linsane.telemetry().addData("I", Util.Constants.Tunable.FLYWHEEL_I);
                        linsane.telemetry().addData("D", Util.Constants.Tunable.FLYWHEEL_D);
                        linsane.telemetry().addData("F @ 13.5V", Util.Constants.Tunable.FLYWHEEL_F);
                    }
                    else { // TRACKING
                        boolean has = Vision.INSTANCE.hasArtifact();
                        double tx = has ? Vision.INSTANCE.getTX() : 0.0;

                        linsane.telemetry().addLine("--- Tracking ---");
                        linsane.telemetry().addData("Pipeline", Util.Constants.Vision.ARTIFACT_PIPELINE);
                        linsane.telemetry().addData("Has Artifact", has);
                        linsane.telemetry().addData("TX", "%.2f deg", tx);
                        linsane.telemetry().addData("Rotate Gain", Util.Constants.Tunable.DRIVE_ROTATE_GAIN);
                        linsane.telemetry().addData("Max Rotate", Util.Constants.Tunable.DRIVE_MAX_ROTATE);
                        linsane.telemetry().addData("Deadband", Util.Constants.Tunable.DRIVE_ARTIFACT_AIM_DEADBAND_DEG);
                    }

                    linsane.telemetry().update();
                }))
        ));

        linsane.dropToScheduler();
    });
}