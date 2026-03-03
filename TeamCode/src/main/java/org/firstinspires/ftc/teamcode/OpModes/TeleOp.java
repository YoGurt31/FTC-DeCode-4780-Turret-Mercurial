package org.firstinspires.ftc.teamcode.OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.SubSystems.DefaultTelemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Flywheel;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Release;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.Util.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@SuppressWarnings("unused")
public final class TeleOp {
    private static Mercurial.RegisterableProgram buildTeleOp(Constants.Field.Alliance alliance, boolean telemetry) {
        return Mercurial.teleop(linsane -> {

            // Hardware Init
            Drive.INSTANCE.setResetPinPointOnInit(false);
            Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Constants.Field.setAlliance(alliance);
            Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Vision.INSTANCE.setPipeline(Constants.Vision.ARTIFACT_PIPELINE);
            Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

            // AprilTag Auto Relocalizer
            final long[] lastRelocalizeMs = {0};
            final long[] stableSinceMs = {0};

            // Elevator
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().options && linsane.gamepad1().share), exec(Elevator.INSTANCE::applyPreset));
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().dpad_up), exec(Elevator.INSTANCE::startRise));

            // Main Loop
            linsane.schedule(sequence(waitUntil(linsane::inLoop), loop(exec(() -> {
                Drive.INSTANCE.updateOdometry();
                Vision.INSTANCE.updateRobotYawDeg(Drive.INSTANCE.getHeading());
                Vision.INSTANCE.update();
                Vision.INSTANCE.setPipeline(Constants.Vision.ARTIFACT_PIPELINE);
                Intake.INSTANCE.setScale(Constants.Field.inFarZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY()) ? Constants.Intake.TRANSFER_SCALE_FAR : Constants.Intake.TRANSFER_SCALE_CLOSE);

                // Drive
                double driveCmd = -linsane.gamepad1().left_stick_y;
                double turnCmd = linsane.gamepad1().right_stick_x;

                // Auto Relocalization
                long nowMs = System.currentTimeMillis();
                Pose3D tagPose = Vision.INSTANCE.getPose();
                boolean stationary = Math.abs(driveCmd) < 0.05 && Math.abs(turnCmd) < 0.05;

                if (stationary && tagPose != null) {
                    if (stableSinceMs[0] == 0) stableSinceMs[0] = nowMs;

                    boolean stableLongEnough = (nowMs - stableSinceMs[0]) >= Constants.Relocalize.STATIONARY_TIME_MS;
                    boolean cooldownOk = (nowMs - lastRelocalizeMs[0]) >= Constants.Relocalize.COOLDOWN_MS;

                    if (stableLongEnough && cooldownOk) {
                        Drive.INSTANCE.relocalizePose(tagPose);
                        lastRelocalizeMs[0] = nowMs;
                        stableSinceMs[0] = 0;
                    }
                } else {
                    stableSinceMs[0] = 0;
                }

                // Intake
                if (linsane.gamepad1().left_bumper) {
                    Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                } else if (linsane.gamepad1().right_bumper) {
                    Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                } else {
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                }

                // Turret - AimBot + Flywheel + Release
                if (linsane.gamepad1().right_trigger >= 0.10) {
                    Pose2D pose = Constants.Field.predictPose(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), Drive.INSTANCE.getHeading(), Drive.INSTANCE.getVx(), Drive.INSTANCE.getVy(), Constants.Ballistic.flyTime(Constants.Field.distanceToGoal(), Flywheel.INSTANCE.getTargetRps()));
                    Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), Constants.Field.computeGoalHeadingDeg(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), alliance)); // Predicted
                    Flywheel.INSTANCE.enableAutoRange();
                    Release.INSTANCE.open();
                } else if (linsane.gamepad1().left_trigger >= 0.10) {
                    Turret.INSTANCE.lockTurret();
                    if (Vision.INSTANCE.getTag() != null && Vision.INSTANCE.getTag().getTargetArea() >= 0.10 && Constants.Drive.isAllowedTagId(Vision.INSTANCE.getTag().getFiducialId(), alliance)) {
                        double headingErrDeg = Vision.INSTANCE.getTag().getTargetXDegrees();
                        double cmd = headingErrDeg * Constants.Drive.KP * Constants.Drive.TX_SIGN;
                        turnCmd = Range.clip(cmd, -Constants.Drive.MAX_TURN, Constants.Drive.MAX_TURN);
                    }
                    Flywheel.INSTANCE.enableAutoRange();
                    Release.INSTANCE.open();
                } else {
                    Turret.INSTANCE.stop();
                    Flywheel.INSTANCE.setVelocityRps(Constants.Flywheel.MIN_RPS);
                    Release.INSTANCE.close();
                }

                Drive.INSTANCE.drive(driveCmd, turnCmd);
                Intake.INSTANCE.apply();
                Flywheel.INSTANCE.apply();
                Release.INSTANCE.apply();
                Elevator.INSTANCE.updateRise();

                if (telemetry) {
                    DefaultTelemetry.INSTANCE.update(linsane.telemetry());
                }
            }))));

            // Shut Off
            linsane.dropToScheduler();
        });
    }

    public static final Mercurial.RegisterableProgram DEBUG = buildTeleOp(Constants.Field.Alliance.BLUE, true);
    public static final Mercurial.RegisterableProgram RED = buildTeleOp(Constants.Field.Alliance.RED, false);
    public static final Mercurial.RegisterableProgram BLUE = buildTeleOp(Constants.Field.Alliance.BLUE, false);
}