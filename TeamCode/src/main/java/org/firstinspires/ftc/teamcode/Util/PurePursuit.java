package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;

import java.util.ArrayList;
import java.util.List;

public class PurePursuit {

    public static class Waypoint {
        public final double x, y;

        public Waypoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    // Core tuning
    public double trackWidthIn = 12.0;
    public double lookaheadIn = 18.0;

    public double maxPower = 1.0;     // you can keep = 1.0
    public double minPower = 0.1;     // used only when far from end

    public double endPosTolIn = 1.0;
    public double slowDownRadiusIn = 36.0;
    public long settleMs = 250;

    // Safety / smoothness
    public double maxCurvature = 1.5;

    // Reduce jitter at high speed
    public double curvatureAlpha = 0.25;       // 0..1, lower = smoother
    public double localYDeadbandIn = 0.50;     // ignore tiny lateral noise

    // Speed regulation (keeps maxPower fast on straights, slows in turns/end to prevent hunting)
    public double curvatureSpeedGain = 1.25;   // higher => more slowdown in turns
    public double endSlowMinScale = 0.12;      // minimum scale inside slowDownRadius
    public double minPowerDisableDistIn = 18.0;// inside this, minPower is disabled so it can settle

    // Optional reverse driving (OFF by default because it causes the 180° flip behavior)
    public boolean allowReverse = false;
    public double reverseTriggerIn = 3.0;

    private final List<Waypoint> path = new ArrayList<>();
    private boolean busy = false;

    // We track path progress by closest point, not just “last intersection”
    private int seg = 0;
    private double segT = 0.0;

    private long settleStartMs = -1;
    private double filteredCurvature = 0.0;

    public void begin(List<Waypoint> points) {
        path.clear();
        path.addAll(points);
        busy = path.size() >= 2;

        seg = 0;
        segT = 0.0;

        settleStartMs = -1;
        filteredCurvature = 0.0;
    }

    public boolean isBusy() {
        return busy;
    }

    public void cancel() {
        busy = false;
        settleStartMs = -1;
        filteredCurvature = 0.0;
        Drive.INSTANCE.stop();
    }

    public boolean step() {
        if (!busy) return false;
        if (path.size() < 2) {
            cancel();
            return false;
        }

        Drive.INSTANCE.updateOdometry();

        double rx = Drive.INSTANCE.getX();
        double ry = Drive.INSTANCE.getY();
        double headingRad = Math.toRadians(Drive.INSTANCE.getHeading());

        Waypoint end = path.get(path.size() - 1);
        double endDist = hypot(end.x - rx, end.y - ry);

        // End condition
        if (endDist <= endPosTolIn) {
            Drive.INSTANCE.stop();
            if (settleStartMs < 0) settleStartMs = System.currentTimeMillis();
            if (System.currentTimeMillis() - settleStartMs >= settleMs) {
                cancel();
                return false;
            }
            return true;
        } else {
            settleStartMs = -1;
        }

        // 1) Update progress to closest point on path (major anti-stall / anti-jitter fix)
        updateProgressToClosestPoint(rx, ry);

        // 2) Find lookahead point ahead of current progress
        Waypoint look = findLookaheadAhead(rx, ry, lookaheadIn);
        if (look == null) look = end;

        // 3) Convert lookahead point to robot frame (robot forward = +X, left = +Y)
        double dx = look.x - rx;
        double dy = look.y - ry;

        double c = Math.cos(-headingRad);
        double s = Math.sin(-headingRad);

        double localX = dx * c - dy * s;
        double localY = dx * s + dy * c;

        // deadband tiny lateral noise (kills straight-line oscillation)
        if (Math.abs(localY) < localYDeadbandIn) localY = 0.0;

        // Ld = actual distance to follow point in robot frame
        double Ld = Math.max(Math.hypot(localX, localY), 1e-6);

        // 4) Standard pure pursuit curvature: k = 2*sin(alpha)/Ld
        double alpha = Math.atan2(localY, localX);
        double curvature = (2.0 * Math.sin(alpha)) / Ld;
        curvature = Range.clip(curvature, -maxCurvature, maxCurvature);

        // smooth curvature to prevent sign-flip jitter
        double a = Range.clip(curvatureAlpha, 0.0, 1.0);
        filteredCurvature += a * (curvature - filteredCurvature);
        curvature = filteredCurvature;

        // 5) Speed command (regulated)
        double v = maxPower;

        // slow down near end
        if (slowDownRadiusIn > 1e-6 && endDist < slowDownRadiusIn) {
            double scale = endDist / slowDownRadiusIn;                 // 0..1
            scale = Range.clip(scale, endSlowMinScale, 1.0);
            // make it softer (less abrupt) near the end
            v *= (scale * scale);
        }

        // slow down in turns (prevents overshoot/hunting while keeping maxPower on straights)
        v *= 1.0 / (1.0 + curvatureSpeedGain * Math.abs(curvature));

        // apply min power only when not near end
        if (endDist > minPowerDisableDistIn) {
            v = applyMinPower(v, minPower);
        } else {
            // let it settle
            v = Range.clip(v, -maxPower, maxPower);
        }

        // Optional reversing (OFF by default)
        if (allowReverse && localX < -reverseTriggerIn) {
            v = -Math.abs(v);
        } else {
            v = Math.abs(v);
        }

        // 6) Convert (v, curvature) -> tank wheel powers
        // omega ~= v * curvature
        double omega = v * curvature;

        double left = v - omega * (trackWidthIn / 2.0);
        double right = v + omega * (trackWidthIn / 2.0);

        // normalize to [-maxPower, maxPower]
        double m = Math.max(Math.abs(left), Math.abs(right));
        if (m > maxPower) {
            left = left / m * maxPower;
            right = right / m * maxPower;
        }

        left = Range.clip(left, -maxPower, maxPower);
        right = Range.clip(right, -maxPower, maxPower);

        Drive.INSTANCE.setLeftPower(left);
        Drive.INSTANCE.setRightPower(right);

        return true;
    }

    private void updateProgressToClosestPoint(double rx, double ry) {
        // Search forward from current segment; this prevents snapping backwards.
        int start = Math.max(0, seg);
        int end = Math.min(path.size() - 2, start + 8); // window search keeps it fast

        double bestDist2 = Double.POSITIVE_INFINITY;
        int bestSeg = seg;
        double bestT = segT;

        for (int i = start; i <= end; i++) {
            Waypoint p1 = path.get(i);
            Waypoint p2 = path.get(i + 1);

            double vx = p2.x - p1.x;
            double vy = p2.y - p1.y;
            double len2 = vx * vx + vy * vy;
            if (len2 < 1e-9) continue;

            double wx = rx - p1.x;
            double wy = ry - p1.y;

            double t = (wx * vx + wy * vy) / len2;
            t = Range.clip(t, 0.0, 1.0);

            double cx = p1.x + t * vx;
            double cy = p1.y + t * vy;

            double dx = rx - cx;
            double dy = ry - cy;
            double d2 = dx * dx + dy * dy;

            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestSeg = i;
                bestT = t;
            }
        }

        seg = bestSeg;
        segT = bestT;

        // If we are basically at the end of this segment, advance.
        if (segT > 0.98 && seg < path.size() - 2) {
            seg++;
            segT = 0.0;
        }
    }

    private Waypoint findLookaheadAhead(double rx, double ry, double radius) {
        // Start from current progress and only look forward.
        for (int i = seg; i < path.size() - 1; i++) {
            Waypoint p1 = path.get(i);
            Waypoint p2 = path.get(i + 1);

            List<Intersection> hits = circleSegmentIntersections(rx, ry, radius, p1, p2);
            if (hits.isEmpty()) continue;

            // Choose the intersection that is "furthest along" this segment.
            Intersection best = hits.get(0);
            if (hits.size() == 2 && hits.get(1).t > best.t) best = hits.get(1);

            // If this is the current segment, make sure we don't pick a point behind our progress.
            if (i == seg && best.t < segT) {
                // try the other hit if it exists
                if (hits.size() == 2) {
                    Intersection other = (best == hits.get(0)) ? hits.get(1) : hits.get(0);
                    if (other.t >= segT) best = other;
                    else continue;
                } else {
                    continue;
                }
            }

            return new Waypoint(best.x, best.y);
        }
        return null;
    }

    private static class Intersection {
        final double x, y, t;

        Intersection(double x, double y, double t) {
            this.x = x;
            this.y = y;
            this.t = t;
        }
    }

    private List<Intersection> circleSegmentIntersections(double cx, double cy, double r, Waypoint p1, Waypoint p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        double fx = p1.x - cx;
        double fy = p1.y - cy;

        double a = dx * dx + dy * dy;
        double b = 2.0 * (fx * dx + fy * dy);
        double c = (fx * fx + fy * fy) - r * r;

        double disc = b * b - 4 * a * c;
        List<Intersection> out = new ArrayList<>();
        if (disc < 0) return out;

        double sqrt = Math.sqrt(disc);
        double t1 = (-b - sqrt) / (2 * a);
        double t2 = (-b + sqrt) / (2 * a);

        if (t1 >= 0.0 && t1 <= 1.0) out.add(new Intersection(p1.x + t1 * dx, p1.y + t1 * dy, t1));
        if (t2 >= 0.0 && t2 <= 1.0) out.add(new Intersection(p1.x + t2 * dx, p1.y + t2 * dy, t2));

        return out;
    }

    private static double applyMinPower(double p, double min) {
        if (Math.abs(p) < 1e-6) return 0.0;
        double sign = Math.signum(p);
        double mag = Math.abs(p);
        if (mag < min) mag = min;
        return sign * mag;
    }

    private static double hypot(double a, double b) {
        return Math.hypot(a, b);
    }
}