package org.firstinspires.ftc.teamcode.OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.parallel;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitSeconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.jumpScope;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.race;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Release;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.PID2Point;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import dev.frozenmilk.dairy.mercurial.continuations.Closure;

@SuppressWarnings("unused")
public final class Auton {

    public static final Mercurial.RegisterableProgram RedFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Constants.Field.setAlliance(Constants.Field.Alliance.RED);
        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
        linsane.schedule(loop(sequence(waitUntil(linsane::inLoop), exec(() -> {
            Drive.INSTANCE.updateOdometry();
            Vision.INSTANCE.update();
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.apply();
            Release.INSTANCE.apply();
        }))));

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                applyStartPose(
                        Constants.Field.Alliance.RED,
                        Constants.Field.StartPose.RED_FAR.START_X_IN,
                        Constants.Field.StartPose.RED_FAR.START_Y_IN,
                        Constants.Field.StartPose.RED_FAR.START_HEADING_DEG
                ),
                buildRedFar(),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.close();
                    Release.INSTANCE.apply();
                })));

        // Shut Off
        linsane.dropToScheduler();
    });

    public static final Mercurial.RegisterableProgram BlueFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Constants.Field.setAlliance(Constants.Field.Alliance.BLUE);
        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
        linsane.schedule(loop(sequence(waitUntil(linsane::inLoop), exec(() -> {
            Drive.INSTANCE.updateOdometry();
            Vision.INSTANCE.update();
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.apply();
            Release.INSTANCE.apply();
        }))));

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                applyStartPose(
                        Constants.Field.Alliance.BLUE,
                        Constants.Field.StartPose.BLUE_FAR.START_X_IN,
                        Constants.Field.StartPose.BLUE_FAR.START_Y_IN,
                        Constants.Field.StartPose.BLUE_FAR.START_HEADING_DEG
                ),
                buildBlueFar(),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.close();
                    Release.INSTANCE.apply();
                })));

        // Shut Off
        linsane.dropToScheduler();
    });

    // XXX: CREATE PATHING
    private static Closure buildRedFar() {

        return sequence(
                shootArtifacts(Constants.Field.Alliance.RED),
                PID2Point.DriveDistance(27.0),
                PID2Point.TurnTo(-90.0),
                driveAndIntakeArtifacts(40.0),
                PID2Point.DriveDistance(-34.0),
                PID2Point.TurnTo(0.0),
                PID2Point.DriveDistance(-20.0),
                shootArtifacts(Constants.Field.Alliance.RED),
                PID2Point.TurnTo(-97.5),
                driveAndIntakeArtifacts(36),
                parallel(
                        PID2Point.TurnTo(-90),
                        intakeArtifacts(2)
                ),
                parallel(
                        PID2Point.TurnTo(-95),
                        intakeArtifacts(1)
                ),
                driveAndIntakeArtifacts(-38),
                shootArtifacts(Constants.Field.Alliance.RED),
                PID2Point.DriveDistance(30.0)
        );
    }

    // XXX: CREATE PATHING
    private static Closure buildBlueFar() {

        return sequence(
                shootArtifacts(Constants.Field.Alliance.BLUE),
                PID2Point.DriveDistance(27.0),
                PID2Point.TurnTo(90.0),
                driveAndIntakeArtifacts(40.0),
                PID2Point.DriveDistance(-34.0),
                PID2Point.TurnTo(0.0),
                PID2Point.DriveDistance(-20.0),
                shootArtifacts(Constants.Field.Alliance.BLUE),
                PID2Point.TurnTo(97.5),
                driveAndIntakeArtifacts(36),
                parallel(
                        PID2Point.TurnTo(90),
                        intakeArtifacts(2)
                ),
                parallel(
                        PID2Point.TurnTo(95),
                        intakeArtifacts(1)
                ),
                driveAndIntakeArtifacts(-38),
                shootArtifacts(Constants.Field.Alliance.BLUE),
                PID2Point.DriveDistance(30.0)
        );
    }

    // XXX: ACTIONS LIBRARY
    private static Closure intakeArtifacts(double seconds) {
        return sequence(
                exec(() -> {
                    Release.INSTANCE.close();
                    Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                    Intake.INSTANCE.apply();
                }),
                waitSeconds(seconds),
                exec(() -> {
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    Intake.INSTANCE.apply();
                })
        );
    }

    private static Closure intakeArtifactsWhileDriving() {
        return sequence(
                exec(() -> {
                    Release.INSTANCE.close();
                    Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                    Intake.INSTANCE.apply();
                }),
                waitUntil(Drive.INSTANCE::isBusy),
                waitUntil(() -> !Drive.INSTANCE.isBusy()),
                exec(() -> {
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    Intake.INSTANCE.apply();
                })
        );
    }

    private static Closure turretAutoAim(Constants.Field.Alliance alliance) {
        return exec(() -> {
            double x = Drive.INSTANCE.getX();
            double y = Drive.INSTANCE.getY();
            double heading = Drive.INSTANCE.getHeading();
            double goalHeading = Constants.Field.computeGoalHeadingDeg(x, y, alliance);
            Release.INSTANCE.open();
            Release.INSTANCE.apply();
            Turret.INSTANCE.autoAimTurret(heading, goalHeading);
            Flywheel.INSTANCE.enableAutoRange();
            Flywheel.INSTANCE.apply();
            Drive.INSTANCE.drive(0.0, 0.0);
        });
    }

    private static Closure waitForReady(Constants.Field.Alliance alliance) {
        return sequence(race(jumpScope(jumpHandle -> loop(ifHuh(() -> Flywheel.INSTANCE.isReady() && Turret.INSTANCE.isAligned(), jumpHandle.jump()).elseHuh(sequence(turretAutoAim(alliance), exec(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        })))))), exec(Drive.INSTANCE::stop));
    }

    private static Closure feedPulse(Constants.Field.Alliance alliance) {
        return race(loop(sequence(turretAutoAim(alliance), exec(() -> {
            if (Flywheel.INSTANCE.isReady() && Turret.INSTANCE.isAligned()) {
                Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            } else {
                Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            }
            Intake.INSTANCE.apply();
        }))), waitSeconds(Constants.Auton.FEED_PULSE_SEC));
    }

    private static Closure stopFeed(Constants.Field.Alliance alliance) {
        return race(loop(sequence(turretAutoAim(alliance), exec(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        }))), waitSeconds(Math.max(0.0, Constants.Auton.SHOT_TOTAL_SEC - Constants.Auton.FEED_PULSE_SEC)));
    }

    private static Closure singleShot(Constants.Field.Alliance alliance) {
        return sequence(
                waitForReady(alliance),
                feedPulse(alliance),
                stopFeed(alliance)
        );
    }

    private static Closure shootingCleanup() {
        return exec(() -> {
            Drive.INSTANCE.drive(0.0, 0.0);
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.stop();
            Turret.INSTANCE.stop();
            Release.INSTANCE.close();
            Release.INSTANCE.apply();
        });
    }

    private static Closure shootArtifacts(Constants.Field.Alliance alliance) {
        return sequence(
                exec(() -> {
                    Drive.INSTANCE.drive(0.0, 0.0);
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    Intake.INSTANCE.apply();
                    Release.INSTANCE.open();
                    Release.INSTANCE.apply();
                }),

                singleShot(alliance),

                sequence(exec(() -> {
                    Drive.INSTANCE.drive(0.0, 0.0);
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    Intake.INSTANCE.apply();
                    Release.INSTANCE.open();
                    Release.INSTANCE.apply();
                })),

                singleShot(alliance),

                sequence(exec(() -> {
                    Drive.INSTANCE.drive(0.0, 0.0);
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    Intake.INSTANCE.apply();
                    Release.INSTANCE.open();
                    Release.INSTANCE.apply();
                })),

                singleShot(alliance),

                shootingCleanup()
        );
    }

    private static Closure driveAndIntakeArtifacts(double distanceIn) {
        return sequence(parallel(intakeArtifactsWhileDriving(), PID2Point.DriveDistance(distanceIn)), exec(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        }));
    }

    private static Closure turnAndIntakeArtifacts(double deg) {
        return sequence(parallel(intakeArtifactsWhileDriving(), PID2Point.TurnTo(deg)), exec(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        }));
    }

    private static Closure applyStartPose(Constants.Field.Alliance alliance, double startXIn, double startYIn, double startHeadingDeg) {
        return exec(() -> {
            Constants.Field.setAlliance(alliance);
            Drive.INSTANCE.setPose(startXIn, startYIn, startHeadingDeg);
            Drive.INSTANCE.updateOdometry();
        });
    }
}