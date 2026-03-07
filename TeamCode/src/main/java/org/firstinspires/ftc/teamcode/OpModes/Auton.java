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

import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.continuations.Closure;

@SuppressWarnings("unused")
public final class Auton {

    public static final Mercurial.RegisterableProgram RedFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Constants.Field.setAlliance(Constants.Field.Alliance.RED);
        Drive.INSTANCE.setPose(Constants.Field.StartPose.RED_FAR.START_X_IN, Constants.Field.StartPose.RED_FAR.START_Y_IN, Constants.Field.StartPose.RED_FAR.START_HEADING_DEG);

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
        linsane.schedule(loop(sequence(waitUntil(linsane::inLoop), exec(() -> {
            Drive.INSTANCE.updateOdometry();
            if (Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY())) Release.INSTANCE.open();
            else Release.INSTANCE.close();
            Vision.INSTANCE.update();
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.apply();
            Release.INSTANCE.apply();
        }))));

        // Autonomous Sequence
        linsane.schedule(sequence(waitUntil(linsane::inLoop), buildRedFar(),
                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.close();
                })));

        // Shut Off
        linsane.dropToScheduler();
    });

    public static final Mercurial.RegisterableProgram BlueFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Constants.Field.setAlliance(Constants.Field.Alliance.BLUE);
        Drive.INSTANCE.setPose(Constants.Field.StartPose.BLUE_FAR.START_X_IN, Constants.Field.StartPose.BLUE_FAR.START_Y_IN, Constants.Field.StartPose.BLUE_FAR.START_HEADING_DEG);

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
        linsane.schedule(loop(sequence(waitUntil(linsane::inLoop), exec(() -> {
            Drive.INSTANCE.updateOdometry();
            if (Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY()))
                Release.INSTANCE.open();
            else Release.INSTANCE.close();
            Vision.INSTANCE.update();
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.apply();
            Release.INSTANCE.apply();
        }))));

        // Autonomous Sequence
        linsane.schedule(sequence(waitUntil(linsane::inLoop), buildBlueFar(),
                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.close();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });

    private static Closure intakeArtifactsAction() {
        Closure intake = race(loop(exec(() -> {
            Release.INSTANCE.close();
            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            Intake.INSTANCE.apply();
        })), waitSeconds(3.00));

        Closure cleanup = exec(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        });

        return sequence(
                intake,
                cleanup
        );
    }

    private static Closure shootArtifactsAction(Constants.Field.Alliance alliance) {

        Runnable autoAimTurret = () -> {
            double x = Drive.INSTANCE.getX();
            double y = Drive.INSTANCE.getY();
            double heading = Drive.INSTANCE.getHeading();
            double goalHeading = Constants.Field.computeGoalHeadingDeg(x, y, alliance);
            Turret.INSTANCE.autoAimTurret(heading, goalHeading);
        };

        Closure spinUpAndAlignUntilReady = sequence(race(jumpScope(jumpHandle -> loop(ifHuh(() -> Flywheel.INSTANCE.isReady() && (!Vision.INSTANCE.hasTag() || Math.abs(Vision.INSTANCE.getTX()) <= 1.0), jumpHandle.jump()).elseHuh(exec(() -> {
            autoAimTurret.run();
            Drive.INSTANCE.drive(0.0, 0.0);
            Flywheel.INSTANCE.enableAutoRange();
            Flywheel.INSTANCE.apply();
        })))), waitSeconds(3.00)), exec(Drive.INSTANCE::stop));

        Runnable holdAimAndSpin = () -> {
            autoAimTurret.run();
            Flywheel.INSTANCE.enableAutoRange();
            Flywheel.INSTANCE.apply();
            Drive.INSTANCE.drive(0.0, 0.0);
        };

        Runnable feedPulse = () -> {
            holdAimAndSpin.run();
            if (Flywheel.INSTANCE.isReady() && ((!Vision.INSTANCE.hasTag()) || Math.abs(Vision.INSTANCE.getTX()) <= 1.0)) {
                Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            } else {
                Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            }
            Intake.INSTANCE.apply();
        };

        Runnable stopFeeder = () -> {
            holdAimAndSpin.run();
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        };

        Closure shoot = sequence(race(loop(exec(feedPulse)), waitSeconds(Constants.Auton.FEED_PULSE_SEC)), race(loop(exec(stopFeeder)), waitSeconds(Math.max(0.0, Constants.Auton.SHOT_TOTAL_SEC - Constants.Auton.FEED_PULSE_SEC))));

        Closure gap = sequence(exec(() -> {
            Drive.INSTANCE.drive(0.0, 0.0);
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        }), waitSeconds(Constants.Auton.GAP_SEC));

        Closure cleanup = exec(() -> {
            Drive.INSTANCE.drive(0.0, 0.0);
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.stop();
            Turret.INSTANCE.stop();
        });

        return sequence(
                spinUpAndAlignUntilReady,
                shoot,
                gap,
                shoot,
                gap,
                shoot,
                cleanup
        );
    }

    private static Closure driveAndIntakeArtifactsAction(double distanceIn) {
        return sequence(
                parallel(
                        exec(() -> {
                            Release.INSTANCE.close();
                            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                            Intake.INSTANCE.apply();
                        }),
                        PID2Point.DriveDistance(distanceIn)
                ),
                exec(() -> {
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                    Intake.INSTANCE.apply();
                })
        );
    }

    // FIXME
    private static Closure buildRedFar() {
        Constants.Field.StartPose sp = Constants.Field.StartPose.RED_FAR;

        Drive.INSTANCE.setPose(sp.START_X_IN, sp.START_Y_IN, sp.START_HEADING_DEG);

        return sequence(
                shootArtifactsAction(Constants.Field.Alliance.RED),
                PID2Point.DriveDistance(12.0),
                PID2Point.TurnTo(-100.0),
                driveAndIntakeArtifactsAction(40.0),
                PID2Point.TurnTo(-90.0),
                PID2Point.DriveDistance(-60.0),
                shootArtifactsAction(Constants.Field.Alliance.RED)
        );
    }

    // FIXME
    private static Closure buildBlueFar() {
        Constants.Field.StartPose sp = Constants.Field.StartPose.BLUE_FAR;

        Drive.INSTANCE.setPose(sp.START_X_IN, sp.START_Y_IN, sp.START_HEADING_DEG);

        return sequence(
                shootArtifactsAction(Constants.Field.Alliance.BLUE),
                PID2Point.DriveDistance(12.0),
                PID2Point.TurnTo(100.0),
                driveAndIntakeArtifactsAction(40.0),
                PID2Point.TurnTo(90.0),
                PID2Point.DriveDistance(-60.0),
                shootArtifactsAction(Constants.Field.Alliance.BLUE)
        );
    }
}