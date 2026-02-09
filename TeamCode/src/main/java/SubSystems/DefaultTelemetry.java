package SubSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DefaultTelemetry {
    public static final DefaultTelemetry INSTANCE = new DefaultTelemetry();

    private DefaultTelemetry() {
    }

    public void update(Telemetry telemetry) {
        if (telemetry == null) return;

        // Drive / Vision
        telemetry.addLine("=== Drive + Vision ===");
        telemetry.addData("X Pos", "%5.2f", Drive.INSTANCE.getX());
        telemetry.addData("Y Pos", "%5.2f", Drive.INSTANCE.getY());
        telemetry.addData("Angle", "%5.2f", Drive.INSTANCE.getHeading());
        telemetry.addData("Tag In View", Vision.INSTANCE.hasTag());
        telemetry.addData("Tag Area", Vision.INSTANCE.getTA());
        telemetry.addData("Current Pipeline", Vision.INSTANCE.getPipelineName());
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
        telemetry.addData("Range Mode", Flywheel.INSTANCE.rangeMode() ? "CLOSE" : "FAR");
        telemetry.addData("Shooter Status", Flywheel.INSTANCE.isReady() ? "READY" : "CHARGING");
        telemetry.addData("Gate", Release.INSTANCE.isGateOpen() ? "Open" : "Closed");
        telemetry.addLine();

        telemetry.update();
    }
}
