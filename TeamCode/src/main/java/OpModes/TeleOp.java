package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.roadrunner.Pose2d;

import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import SubSystems.DefaultTelemetry;
import SubSystems.Drive;
import SubSystems.Elevator;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Release;
import SubSystems.Turret;
import SubSystems.Vision;
import Util.Constants;

@SuppressWarnings("unused")
public final class TeleOp {
    private static Mercurial.RegisterableProgram buildTeleOp(Constants.Field.Alliance alliance, boolean telemetry) {
        return Mercurial.teleop(linsane -> {

            // Hardware Init
            Drive.INSTANCE.setResetPinPointOnInit(false);
            Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Constants.Field.setAlliance(alliance);
            Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Vision.INSTANCE.setPipeline(Constants.Vision.LOCALIZATION_PIPELINE);
            Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

            final boolean[] wasShooting = {false};
            final long[] lastMs = {0};
            final long[] stationarySinceMs = {0};

            // Elevator
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().options && linsane.gamepad1().share), exec(Elevator.INSTANCE::applyPreset));
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().dpad_up), exec(Elevator.INSTANCE::startRise));

            // Main Loop
            linsane.schedule(sequence(waitUntil(linsane::inLoop), loop(exec(() -> {
                Drive.INSTANCE.updateOdometry();
                Vision.INSTANCE.updateRobotYawDeg(Drive.INSTANCE.getHeading());
                Vision.INSTANCE.update();

                boolean shootingNow = linsane.gamepad1().right_trigger >= 0.10;
                boolean intakingNow = linsane.gamepad1().right_bumper && !linsane.gamepad1().left_bumper;

                Intake.INSTANCE.setScale(Constants.Field.inFarZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY()) ? Constants.Intake.TRANSFER_SCALE_FAR : Constants.Intake.TRANSFER_SCALE_CLOSE);
                Vision.INSTANCE.setPipeline(intakingNow ? Constants.Vision.ARTIFACT_PIPELINE : Constants.Vision.LOCALIZATION_PIPELINE);

                // Intake
                if (linsane.gamepad1().left_bumper) {
                    Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                } else if (linsane.gamepad1().right_bumper) {
                    Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                } else {
                    Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                }

                // Turret + Flywheel + Release
                if (linsane.gamepad1().right_trigger >= 0.10) {
                    Pose2d pose = Constants.Field.predictPose(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), Math.toRadians(Drive.INSTANCE.getHeading()), Drive.INSTANCE.getVx(), Drive.INSTANCE.getVy(), Constants.Ballistic.flyTime(Constants.Field.distanceToGoal(), Flywheel.INSTANCE.getTargetRps()));
                    Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), Constants.Field.computeGoalHeadingDeg(pose.position.x, pose.position.y, alliance)); // Predicted
                    Flywheel.INSTANCE.enableAutoRange();
                    Release.INSTANCE.open();
                } else if (linsane.gamepad1().left_trigger >= 0.10) {
                    Turret.INSTANCE.lockTurret();
                    Flywheel.INSTANCE.enableAutoRange();
                    Release.INSTANCE.open();
                } else {
                    Turret.INSTANCE.stop();
                    Flywheel.INSTANCE.setVelocityRps(Constants.Flywheel.MIN_RPS);
                    Release.INSTANCE.close();
                }

                // Drive
                double driveCmd = -linsane.gamepad1().left_stick_y;
                double turnCmd = linsane.gamepad1().right_stick_x;
                double rawDrive = driveCmd, rawTurn = turnCmd;

                if (intakingNow) {
                    if (Vision.INSTANCE.hasArtifact()) {
                        double tx = Vision.INSTANCE.getTX();
                        if (Math.abs(tx) <= Constants.Drive.ARTIFACT_AIM_DEADBAND_DEG) {
                            turnCmd = 0.0;
                        } else {
                            double assist = tx * Constants.Drive.ROTATE_GAIN;
                            if (assist > Constants.Drive.MAX_ROTATE)
                                assist = Constants.Drive.MAX_ROTATE;
                            if (assist < -Constants.Drive.MAX_ROTATE)
                                assist = -Constants.Drive.MAX_ROTATE;
                            turnCmd = assist;
                        }
                    }
                }

                // Relocalize
                boolean justStoppedShooting = wasShooting[0] && !shootingNow;
                wasShooting[0] = shootingNow;
                long now = System.currentTimeMillis();
                boolean stationary = Math.abs(rawDrive) < Constants.Relocalize.STATIONARY_STICK_DB && Math.abs(rawTurn) < Constants.Relocalize.STATIONARY_STICK_DB;
                if (stationary) {
                    if (stationarySinceMs[0] == 0) stationarySinceMs[0] = now;
                } else {
                    stationarySinceMs[0] = 0;
                }
                boolean stationaryLongEnough = stationarySinceMs[0] != 0 && (now - stationarySinceMs[0]) >= Constants.Relocalize.STATIONARY_TIME_MS;
                boolean cooldownOk = (now - lastMs[0]) > Constants.Relocalize.COOLDOWN_MS;
                boolean inLocalizationPipe = Vision.INSTANCE.getPipelineIndex() == Constants.Vision.LOCALIZATION_PIPELINE;
                boolean shouldRelocalize = (stationaryLongEnough || (justStoppedShooting && stationary));

                if (shouldRelocalize && cooldownOk && inLocalizationPipe && Vision.INSTANCE.hasPose()) {
                    if (Vision.INSTANCE.getPose() != null) {
                        Drive.INSTANCE.relocalizePose(Vision.INSTANCE.getPose());
                        Turret.INSTANCE.onRobotPoseReset();
                        lastMs[0] = now;
                        stationarySinceMs[0] = now;
                    }
                }

                if (linsane.gamepad1().psWasPressed()) {
                    Drive.INSTANCE.setPose(-61, (Constants.Field.getAlliance() == Constants.Field.Alliance.BLUE) ? -65 : 65, (Constants.Field.getAlliance() == Constants.Field.Alliance.BLUE) ? 88 : -88);
                    Turret.INSTANCE.onRobotPoseReset();
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