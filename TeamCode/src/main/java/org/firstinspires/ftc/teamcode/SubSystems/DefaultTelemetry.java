package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Util.Constants;

public class DefaultTelemetry {
    public static final DefaultTelemetry INSTANCE = new DefaultTelemetry();
    private static final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private DefaultTelemetry() {
    }

    public void update(Telemetry telemetry) {
        if (telemetry == null) return;

        // Drive
        telemetry.addLine("=== Localization ===");
        telemetry.addData("X Pos", "%5.2f", Drive.INSTANCE.getX());
        telemetry.addData("Y Pos", "%5.2f", Drive.INSTANCE.getY());
        telemetry.addData("Angle", "%5.2f", Drive.INSTANCE.getHeading());
        telemetry.addData("Distance To Goal", "%5.2f", Constants.Field.distanceToGoal());
        telemetry.addData("In Shoot Zone", Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY()));
        telemetry.addLine();

        // Vision
        telemetry.addLine("=== Vision ===");
        telemetry.addLine("--- Limelight (AprilTag) ---");
        telemetry.addData("Pipeline", Vision.INSTANCE.getPipelineName());
        telemetry.addData("Has Tag", Vision.INSTANCE.hasTag());
        telemetry.addData("Tag ID", Vision.INSTANCE.getTagId());
        Pose3D llPose = Vision.INSTANCE.getPose();
        String llPoseStr = "NULL";
        try {
            if (llPose != null && llPose.getPosition() != null && llPose.getOrientation() != null) {
                double xIn = llPose.getPosition().x * Constants.Relocalize.METERS_TO_IN;
                double yIn = llPose.getPosition().y * Constants.Relocalize.METERS_TO_IN;
                double zIn = llPose.getPosition().z * Constants.Relocalize.METERS_TO_IN;

                double yawDeg = llPose.getOrientation().getYaw(AngleUnit.DEGREES);
                double pitchDeg = llPose.getOrientation().getPitch(AngleUnit.DEGREES);
                double rollDeg = llPose.getOrientation().getRoll(AngleUnit.DEGREES);

                llPoseStr = String.format("x=%.1f in y=%.1f in z=%.1f in | yaw=%.1f pitch=%.1f roll=%.1f", xIn, yIn, zIn, yawDeg, pitchDeg, rollDeg);
            }
        } catch (Exception ignored) { llPoseStr = "INVALID"; }
        telemetry.addData("BotPose", llPoseStr);
        telemetry.addData("TX", "%5.2f", Vision.INSTANCE.getTX());
        telemetry.addData("TY", "%5.2f", Vision.INSTANCE.getTY());
        telemetry.addData("TA", "%5.2f", Vision.INSTANCE.getTA());
        telemetry.addLine();

        // Intake
        telemetry.addLine("=== Intake ===");
        boolean intakeActive = Intake.INSTANCE.getMode() != Intake.Mode.IDLE;
        telemetry.addData("Roller Status", intakeActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Sorting", Intake.INSTANCE.getSortingStatus());
        telemetry.addLine();

        // Turret
        telemetry.addLine("=== Turret ===");
        telemetry.addData("Current Deg", "%6.2f", Turret.INSTANCE.getTurretDeg());
        telemetry.addData("Target Deg", "%6.2f", Turret.INSTANCE.getTargetDeg());
        telemetry.addData("Turret Error", "%6.2f", Turret.INSTANCE.getErrorDeg());
        telemetry.addData("At Min", Turret.INSTANCE.atMinLimit());
        telemetry.addData("At Max", Turret.INSTANCE.atMaxLimit());
        telemetry.addLine();

        // Flywheel / Release
        telemetry.addLine("=== Flywheel + Release ===");
        telemetry.addData("Flywheel1 RPS", "%5.2f", Flywheel.INSTANCE.getRps1());
        telemetry.addData("Flywheel2 RPS", "%5.2f", Flywheel.INSTANCE.getRps2());
        telemetry.addData("Avg RPS", "%5.2f", Flywheel.INSTANCE.getAverageRps());
        telemetry.addData("Target RPS", "%5.2f", Flywheel.INSTANCE.getTargetRps());
        telemetry.addData("Shooter Status", Flywheel.INSTANCE.isReady() ? "READY" : "CHARGING");
        telemetry.addData("Gate", Release.INSTANCE.isGateOpen() ? "Open" : "Closed");
        telemetry.addLine();

        panels.update(telemetry);
        telemetry.update();
    }
}
