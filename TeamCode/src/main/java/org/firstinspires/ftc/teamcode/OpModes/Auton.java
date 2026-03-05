package org.firstinspires.ftc.teamcode.OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.parallel;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitSeconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.jumpScope;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.race;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Release;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.Util.Constants;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.continuations.Closure;

import org.firstinspires.ftc.teamcode.Util.PID2Point;

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
        Intake.INSTANCE.setScale(0.5);
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
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
            Elevator.INSTANCE.updateRise();
        }))));

        // Autonomous Sequence
        linsane.schedule(sequence(waitUntil(linsane::inLoop), buildMain(new PID2Point(), Constants.Field.getAlliance()),
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
        Intake.INSTANCE.setScale(0.5);
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
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
            Elevator.INSTANCE.updateRise();
        }))));

        // Autonomous Sequence
        linsane.schedule(sequence(waitUntil(linsane::inLoop), buildMain(new PID2Point(), Constants.Field.getAlliance()),

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

    private static Closure p2pTo(PID2Point p2p, double xIn, double yIn, double headingDeg) {
        return jumpScope(jumpHandle -> sequence(exec(() -> p2p.beginAbs(xIn, yIn, headingDeg)), loop(ifHuh(p2p::step, noop()).elseHuh(jumpHandle.jump())), exec(Drive.INSTANCE::stop)));
    }

    private static Closure p2pGoTo(PID2Point p2p, double forwardIn, double leftIn, double endHeadingDeg) {
        return jumpScope(jumpHandle -> sequence(exec(() -> {
            Drive.INSTANCE.updateOdometry();
            double x0 = Drive.INSTANCE.getX();
            double y0 = Drive.INSTANCE.getY();
            double h0 = Drive.INSTANCE.getHeading();
            double hRad = Math.toRadians(h0);

            double tx = x0 + forwardIn * Math.cos(hRad) - leftIn * Math.sin(hRad);
            double ty = y0 + forwardIn * Math.sin(hRad) + leftIn * Math.cos(hRad);

            p2p.beginAbs(tx, ty, endHeadingDeg);
        }), loop(ifHuh(p2p::step, noop()).elseHuh(jumpHandle.jump())), exec(Drive.INSTANCE::stop)));
    }

    private static Closure p2pTurnTo(PID2Point p2p, double targetHeadingDeg) {
        return jumpScope(jumpHandle -> sequence(exec(() -> {
            Drive.INSTANCE.updateOdometry();
            double x0 = Drive.INSTANCE.getX();
            double y0 = Drive.INSTANCE.getY();
            p2p.beginAbs(x0, y0, targetHeadingDeg);
        }), loop(ifHuh(p2p::step, noop()).elseHuh(jumpHandle.jump())), exec(Drive.INSTANCE::stop)));
    }

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

        return sequence(intake, cleanup);
    }

    private static Closure shootArtifactsAction(Constants.Field.Alliance alliance) {

        Runnable lockTurret = () -> {
            if (alliance == Constants.Field.Alliance.BLUE) {
                Turret.INSTANCE.lockTurretAt(15.0);
            } else {
                Turret.INSTANCE.lockTurretAt(-15.0);
            }
        };

        Closure spinUpAndAlignUntilReady = sequence(race(jumpScope(jumpHandle -> loop(ifHuh(() -> Flywheel.INSTANCE.isReady() && (!Vision.INSTANCE.hasTag() || Math.abs(Vision.INSTANCE.getTX()) <= 1.0), jumpHandle.jump()).elseHuh(exec(() -> {
            lockTurret.run();

            double turn = Range.clip((Vision.INSTANCE.getTX() * Constants.Drive.KP), -Constants.Drive.MAX_TURN, Constants.Drive.MAX_TURN);

            if (!Vision.INSTANCE.hasTag() || Math.abs(Vision.INSTANCE.getTX()) <= 1.0) {
                turn = 0.0;
            }

            Drive.INSTANCE.drive(0.0, turn);

            Flywheel.INSTANCE.enableAutoRange();
            Flywheel.INSTANCE.apply();
        })))), waitSeconds(2.25)), exec(Drive.INSTANCE::stop));

        Runnable holdAimAndSpin = () -> {
            lockTurret.run();
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

        Closure shot = sequence(race(loop(exec(feedPulse)), waitSeconds(Constants.Auton.FEED_PULSE_SEC)), race(loop(exec(stopFeeder)), waitSeconds(Math.max(0.0, Constants.Auton.SHOT_TOTAL_SEC - Constants.Auton.FEED_PULSE_SEC))));

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

        return sequence(spinUpAndAlignUntilReady, shot, gap, shot, gap, shot, cleanup);
    }

    private static Closure buildMain(PID2Point p2p, Constants.Field.Alliance alliance) {
        Constants.Field.StartPose sp = (alliance == Constants.Field.Alliance.BLUE) ? Constants.Field.StartPose.BLUE_FAR : Constants.Field.StartPose.RED_FAR;
        Drive.INSTANCE.setPose(sp.START_X_IN, sp.START_Y_IN, sp.START_HEADING_DEG);
        return (alliance == Constants.Field.Alliance.BLUE) ? blueFar(p2p, alliance) : redFar(p2p, alliance);
    }

    // TODO: BlueFar Path
    private static Closure blueFar(PID2Point p2p, Constants.Field.Alliance alliance) {
        Closure driveAndIntakeLine1 = parallel(sequence(p2pTo(p2p, -36, 24, 90), p2pTo(p2p, -36, 54, 90)), intakeArtifactsAction());
        Closure driveAndIntakeHPZ = parallel(p2pTo(p2p, -60, 54, 90), intakeArtifactsAction());

        return sequence(shootArtifactsAction(alliance), driveAndIntakeLine1, p2pTo(p2p, -60, 12, 180), shootArtifactsAction(alliance), driveAndIntakeHPZ, p2pTo(p2p, -60, 12, 180), shootArtifactsAction(alliance));
    }

    // TODO: RedFar Path
    private static Closure redFar(PID2Point p2p, Constants.Field.Alliance alliance) {
        Closure driveAndIntakeLine1 = parallel(sequence(p2pTo(p2p, -36, -24, -90), p2pTo(p2p, -36, -54, -90)), intakeArtifactsAction());
        Closure driveAndIntakeHPZ = parallel(p2pTo(p2p, -60, -54, -90), intakeArtifactsAction());

        return sequence(shootArtifactsAction(alliance), driveAndIntakeLine1, p2pTo(p2p, -60, -12, 180), shootArtifactsAction(alliance), driveAndIntakeHPZ, p2pTo(p2p, -60, -12, 180), shootArtifactsAction(alliance));
    }
}