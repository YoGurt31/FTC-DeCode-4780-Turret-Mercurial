package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Pure Pursuit Tuner", group = "Tuning")
public class PPTuner extends LinearOpMode {

    private static class Edge {
        private boolean last = false;
        boolean rising(boolean now) {
            boolean r = now && !last;
            last = now;
            return r;
        }
    }

    private static class PathDef {
        final String name;
        final List<PurePursuit.Waypoint> points;
        PathDef(String name, List<PurePursuit.Waypoint> points) {
            this.name = name;
            this.points = points;
        }
    }

    @Override
    public void runOpMode() {
        Drive.INSTANCE.init(hardwareMap, telemetry);

        PurePursuit pp = new PurePursuit();

        double trackWidthIn = Constants.PurePursuit.TRACK_WIDTH_IN;
        double lookaheadIn = Constants.PurePursuit.LOOKAHEAD_IN;
        double minPower = Constants.PurePursuit.MIN_POWER;
        double maxPower = Constants.PurePursuit.MAX_POWER;
        double endPosTolIn = Constants.PurePursuit.END_POS_TOL_IN;
        double slowDownRadiusIn = Constants.PurePursuit.SLOW_DOWN_RADIUS_IN;
        double maxCurvature = Constants.PurePursuit.MAX_CURVATURE;
        long settleMs = Constants.PurePursuit.SETTLE_MS;

        List<PathDef> paths = buildPaths();
        int pathIndex = 0;

        int paramIndex = 0;
        String[] paramNames = new String[] {
                "trackWidthIn",
                "lookaheadIn",
                "maxPower",
                "minPower",
                "slowDownRadiusIn",
                "endPosTolIn",
                "settleMs",
                "maxCurvature"
        };

        Edge a = new Edge();
        Edge b = new Edge();
        Edge x = new Edge();
        Edge y = new Edge();
        Edge du = new Edge();
        Edge dd = new Edge();
        Edge dl = new Edge();
        Edge dr = new Edge();
        Edge back = new Edge();

        boolean running = false;

        telemetry.addLine("Pure Pursuit Tuner ready");
        telemetry.addLine("A: start path  |  B: stop");
        telemetry.addLine("X/Y: prev/next path");
        telemetry.addLine("Dpad Up/Down: select parameter");
        telemetry.addLine("Dpad Left/Right: adjust (step)");
        telemetry.addLine("LB/RB: smaller/bigger step size");
        telemetry.addLine("Back: reset robot pose to (0,0,0)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Drive.INSTANCE.updateOdometry();

            if (back.rising(gamepad1.back)) {
                Drive.INSTANCE.setPose(0, 0, 0);
                running = false;
                pp.cancel();
            }

            if (x.rising(gamepad1.x)) {
                pathIndex = (pathIndex - 1 + paths.size()) % paths.size();
                running = false;
                pp.cancel();
            }
            if (y.rising(gamepad1.y)) {
                pathIndex = (pathIndex + 1) % paths.size();
                running = false;
                pp.cancel();
            }

            if (du.rising(gamepad1.dpad_up)) {
                paramIndex = (paramIndex - 1 + paramNames.length) % paramNames.length;
            }
            if (dd.rising(gamepad1.dpad_down)) {
                paramIndex = (paramIndex + 1) % paramNames.length;
            }

            double baseStep = gamepad1.right_bumper ? 0.5 : (gamepad1.left_bumper ? 0.05 : 0.1);

            if (dr.rising(gamepad1.dpad_right)) {
                switch (paramIndex) {
                    case 0: trackWidthIn += baseStep; break;
                    case 1: lookaheadIn += baseStep; break;
                    case 2: maxPower += 0.02; break;
                    case 3: minPower += 0.02; break;
                    case 4: slowDownRadiusIn += baseStep; break;
                    case 5: endPosTolIn += 0.25; break;
                    case 6: settleMs += 25; break;
                    case 7: maxCurvature += 0.1; break;
                }
            }

            if (dl.rising(gamepad1.dpad_left)) {
                switch (paramIndex) {
                    case 0: trackWidthIn -= baseStep; break;
                    case 1: lookaheadIn -= baseStep; break;
                    case 2: maxPower -= 0.02; break;
                    case 3: minPower -= 0.02; break;
                    case 4: slowDownRadiusIn -= baseStep; break;
                    case 5: endPosTolIn -= 0.25; break;
                    case 6: settleMs -= 25; break;
                    case 7: maxCurvature -= 0.1; break;
                }
            }

            trackWidthIn = clamp(trackWidthIn, 6.0, 30.0);
            lookaheadIn = clamp(lookaheadIn, 2.0, 30.0);
            maxPower = clamp(maxPower, 0.05, 1.0);
            minPower = clamp(minPower, 0.0, 0.6);
            slowDownRadiusIn = clamp(slowDownRadiusIn, 2.0, 60.0);
            endPosTolIn = clamp(endPosTolIn, 0.25, 6.0);
            settleMs = clampLong(settleMs, 0, 2000);
            maxCurvature = clamp(maxCurvature, 0.2, 6.0);

            if (a.rising(gamepad1.a)) {
                pp.trackWidthIn = trackWidthIn;
                pp.lookaheadIn = lookaheadIn;
                pp.maxPower = maxPower;
                pp.minPower = minPower;
                pp.slowDownRadiusIn = slowDownRadiusIn;
                pp.endPosTolIn = endPosTolIn;
                pp.settleMs = settleMs;
                pp.maxCurvature = maxCurvature;

                pp.begin(paths.get(pathIndex).points);
                running = true;
            }

            if (b.rising(gamepad1.b)) {
                running = false;
                pp.cancel();
                Drive.INSTANCE.stop();
            }

            boolean busy = false;
            if (running) {
                busy = pp.step();
                if (!busy) running = false;
            }

            double xIn = Drive.INSTANCE.getX();
            double yIn = Drive.INSTANCE.getY();
            double hDeg = Drive.INSTANCE.getHeading();

            telemetry.addLine("Controls: A start | B stop | X/Y path | Dpad Up/Down select | Left/Right adjust | LB/RB step");
            telemetry.addData("Path", "%d/%d  %s", pathIndex + 1, paths.size(), paths.get(pathIndex).name);
            telemetry.addData("Running", "%s  busy=%s", running, busy);
            telemetry.addLine();

            telemetry.addData("Pose (in,deg)", "x=%.2f  y=%.2f  h=%.1f", xIn, yIn, hDeg);
            telemetry.addLine();

            telemetry.addData("Selected Param", "%s", paramNames[paramIndex]);
            telemetry.addData("trackWidthIn", "%.2f", trackWidthIn);
            telemetry.addData("lookaheadIn", "%.2f", lookaheadIn);
            telemetry.addData("maxPower", "%.2f", maxPower);
            telemetry.addData("minPower", "%.2f", minPower);
            telemetry.addData("slowDownRadiusIn", "%.2f", slowDownRadiusIn);
            telemetry.addData("endPosTolIn", "%.2f", endPosTolIn);
            telemetry.addData("settleMs", "%d", settleMs);
            telemetry.addData("maxCurvature", "%.2f", maxCurvature);

            telemetry.update();

            idle();
        }
    }

    private List<PathDef> buildPaths() {
        List<PathDef> out = new ArrayList<>();

        out.add(new PathDef(
                "Straight 48in",
                Arrays.asList(
                        new PurePursuit.Waypoint(0, 0),
                        new PurePursuit.Waypoint(48, 0)
                )
        ));

        out.add(new PathDef(
                "L Turn 24x24",
                Arrays.asList(
                        new PurePursuit.Waypoint(0, 0),
                        new PurePursuit.Waypoint(24, 0),
                        new PurePursuit.Waypoint(24, 24)
                )
        ));

        out.add(new PathDef(
                "Gentle Arc",
                Arrays.asList(
                        new PurePursuit.Waypoint(0, 0),
                        new PurePursuit.Waypoint(24, 0),
                        new PurePursuit.Waypoint(48, 24)
                )
        ));

        out.add(new PathDef(
                "S Curve",
                Arrays.asList(
                        new PurePursuit.Waypoint(0, 0),
                        new PurePursuit.Waypoint(24, 18),
                        new PurePursuit.Waypoint(48, -18),
                        new PurePursuit.Waypoint(72, 0)
                )
        ));

        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    private static long clampLong(long v, long lo, long hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
