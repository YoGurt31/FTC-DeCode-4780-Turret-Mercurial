package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

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
            if (Vision.INSTANCE.getLimelight() != null) {
                Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
            }
            Vision.INSTANCE.setTrackedTag(alliance == Constants.Field.Alliance.RED ? Constants.Vision.RED_TAG_ID : Constants.Vision.BLUE_TAG_ID);
            Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

            // Main Loop
            linsane.schedule(sequence(
                    waitUntil(linsane::inLoop),
                    loop(exec(() -> {
                        Vision.INSTANCE.update();
                        Drive.INSTANCE.updateOdometry();

                        // Turret
                        Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), Constants.Field.computeGoalHeadingDeg(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), alliance));
                        // Turret.INSTANCE.lockTurret();

                        // Intake
                        if (linsane.gamepad1().left_bumper) {
                            Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                        } else if (linsane.gamepad1().right_bumper) {
                            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                        } else {
                            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                        }
                        Intake.INSTANCE.setScale(Constants.Field.inFarZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY()) ? Constants.Intake.TRANSFER_SCALE_FAR : Constants.Intake.TRANSFER_SCALE_CLOSE);

                        // Flywheel
                        if (linsane.gamepad1().right_trigger > 0.05) {
                            // Flywheel.INSTANCE.enableAutoRange();
                            Flywheel.INSTANCE.setVelocityRps(67);
                        } else {
                            Flywheel.INSTANCE.stop();
                        }

                        // Release
                        if (linsane.gamepad1().right_trigger > 0.05) {
                            Release.INSTANCE.open();
                        } else {
                            Release.INSTANCE.close();
                        }

                        // Drive
                        double driveCmd = -linsane.gamepad1().left_stick_y;
                        double turnCmd;

                        if (Intake.INSTANCE.getMode() == Intake.Mode.INTAKE) {
                            if (Vision.INSTANCE.hasArtifact()) {
                                double tx = Vision.INSTANCE.getTX();
                                if (Math.abs(tx) <= Constants.Drive.ARTIFACT_AIM_DEADBAND_DEG) {
                                    turnCmd = 0.0;
                                } else {
                                    double assist = tx * Constants.Drive.ROTATE_GAIN;
                                    if (assist > Constants.Drive.MAX_ROTATE) assist = Constants.Drive.MAX_ROTATE;
                                    if (assist < -Constants.Drive.MAX_ROTATE) assist = -Constants.Drive.MAX_ROTATE;
                                    turnCmd = assist;
                                }
                            } else {
                                turnCmd = linsane.gamepad1().right_stick_x;
                            }
                        } else {
                            turnCmd = linsane.gamepad1().right_stick_x;
                        }

                        Drive.INSTANCE.drive(driveCmd, turnCmd);

                        Intake.INSTANCE.apply();
                        Flywheel.INSTANCE.apply();
                        Release.INSTANCE.update();
                        Elevator.INSTANCE.updateRise();

                        if (telemetry) {
                            DefaultTelemetry.INSTANCE.update(linsane.telemetry());
                        }
                    }))
            ));

            // Elevator
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().options && linsane.gamepad1().share),
                    exec(Elevator.INSTANCE::applyPreset)
            );
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().dpad_up),
                    exec(Elevator.INSTANCE::startRise)
            );

            // Shut Off
            linsane.dropToScheduler();
        });
    }

    public static final Mercurial.RegisterableProgram DEBUG = buildTeleOp(Constants.Field.Alliance.BLUE, true);
    public static final Mercurial.RegisterableProgram RED = buildTeleOp(Constants.Field.Alliance.RED, false);
    public static final Mercurial.RegisterableProgram BLUE = buildTeleOp(Constants.Field.Alliance.BLUE, false);
}