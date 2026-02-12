package SubSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DefaultTelemetry {
    public static final DefaultTelemetry INSTANCE = new DefaultTelemetry();

    private DefaultTelemetry() {
    }

    public void update(Telemetry telemetry) {
        if (telemetry == null) return;

        // Drive
        telemetry.addLine("=== Drive ===");
        telemetry.addData("X Pos", "%5.2f", Drive.INSTANCE.getX());
        telemetry.addData("Y Pos", "%5.2f", Drive.INSTANCE.getY());
        telemetry.addData("Angle", "%5.2f", Drive.INSTANCE.getHeading());
        telemetry.addLine();

        // Vision
        telemetry.addLine("=== Vision ===");
        telemetry.addLine("Limelight (Artifacts):");
        telemetry.addData("Pipeline", Vision.INSTANCE.getPipelineName());
        telemetry.addData("Has Artifact", Vision.INSTANCE.hasArtifact());
        telemetry.addData("TX", "%5.2f", Vision.INSTANCE.getTX());
        telemetry.addData("TY", "%5.2f", Vision.INSTANCE.getTY());
        telemetry.addData("TA", "%5.2f", Vision.INSTANCE.getTA());
        telemetry.addLine("TurretCam (AprilTag):");
        telemetry.addData("Tracked Tag ID", Vision.INSTANCE.getTrackedTag());
        telemetry.addData("Has Tracked Tag", Vision.INSTANCE.hasTrackedTag());
        telemetry.addData("Yaw (Deg)", "%5.2f", Vision.INSTANCE.getTrackedYawDeg());
        telemetry.addData("Range", "%5.2f", Vision.INSTANCE.getTrackedRange());
        telemetry.addLine();

        // Intake
        telemetry.addLine("=== Intake ===");
        boolean intakeActive = Intake.INSTANCE.getMode() != Intake.Mode.IDLE;
        telemetry.addData("Roller Status", intakeActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Sorting", Intake.INSTANCE.getSortingStatus());
        telemetry.addLine();

        // Turret
        telemetry.addLine("=== Turret ===");
        telemetry.addData("Turret Deg", "%6.2f", Turret.INSTANCE.getTurretDeg());
        telemetry.addData("Aim Mode", Turret.INSTANCE.getAimModeName());
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

        telemetry.update();
    }
}
