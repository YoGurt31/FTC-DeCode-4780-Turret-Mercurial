package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.SubSystems.Drive;

import java.util.ArrayList;
import java.util.List;

public class PurePursuit {

    public static class Waypoint {
        public final double x, y;
        public Waypoint(double x, double y) { this.x = x; this.y = y; }
    }

    public double trackWidthIn = Constants.PurePursuit.TRACK_WIDTH_IN;
    public double lookaheadIn = Constants.PurePursuit.LOOKAHEAD_IN;
    public double minPower = Constants.PurePursuit.MIN_POWER;
    public double maxPower = Constants.PurePursuit.MAX_POWER;
    public double endPosTolIn = Constants.PurePursuit.END_POS_TOL_IN;
    public double slowDownRadiusIn = Constants.PurePursuit.SLOW_DOWN_RADIUS_IN;
    public double maxCurvature = Constants.PurePursuit.MAX_CURVATURE;
    public long settleMs = Constants.PurePursuit.SETTLE_MS;

    private final List<Waypoint> path = new ArrayList<>();
    private boolean busy = false;
    private int segmentIndex = 0;
    private long settleStartMs = -1;

    public void begin(List<Waypoint> points) {
        path.clear();
        path.addAll(points);
        segmentIndex = 0;
        busy = path.size() >= 2;
        settleStartMs = -1;
    }

    public boolean isBusy() { return busy; }

    public void cancel() {
        busy = false;
        settleStartMs = -1;
        Drive.INSTANCE.stop();
    }

    public boolean step() {
        if (!busy) return false;
        if (path.size() < 2) { cancel(); return false; }

        Drive.INSTANCE.updateOdometry();

        double rx = Drive.INSTANCE.getX();
        double ry = Drive.INSTANCE.getY();
        double headingDeg = Drive.INSTANCE.getHeading();
        double headingRad = Math.toRadians(headingDeg);

        Waypoint end = path.get(path.size() - 1);
        double endDist = hypot(end.x - rx, end.y - ry);

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

        Waypoint look = findLookahead(rx, ry, lookaheadIn);
        if (look == null) {
            look = end;
        }

        double dx = look.x - rx;
        double dy = look.y - ry;

        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);

        double localX = dx * cos - dy * sin;
        double localY = dx * sin + dy * cos;

        double L = Math.max(lookaheadIn, 1e-6);

        double curvature = (2.0 * localY) / (L * L);
        curvature = Range.clip(curvature, -maxCurvature, maxCurvature);

        double power = maxPower;
        if (endDist < slowDownRadiusIn) {
            double scale = Range.clip(endDist / slowDownRadiusIn, 0.15, 1.0);
            power *= scale;
        }
        power = applyMinPower(power, minPower);

        double omega = power * curvature;
        double left = power - omega * (trackWidthIn / 2.0);
        double right = power + omega * (trackWidthIn / 2.0);

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        left = Range.clip(left, -maxPower, maxPower);
        right = Range.clip(right, -maxPower, maxPower);

        Drive.INSTANCE.setLeftPower(left);
        Drive.INSTANCE.setRightPower(right);

        return true;
    }

    private Waypoint findLookahead(double rx, double ry, double radius) {
        Waypoint best = null;

        for (int i = segmentIndex; i < path.size() - 1; i++) {
            Waypoint p1 = path.get(i);
            Waypoint p2 = path.get(i + 1);

            List<Intersection> hits = circleSegmentIntersections(rx, ry, radius, p1, p2);
            if (hits.isEmpty()) continue;

            Intersection bestHit = hits.get(0);
            if (hits.size() == 2 && hits.get(1).t > bestHit.t) bestHit = hits.get(1);

            best = new Waypoint(bestHit.x, bestHit.y);

            if (bestHit.t > 0.8) segmentIndex = i;
            return best;
        }

        return null;
    }

    private static class Intersection {
        final double x, y, t;
        Intersection(double x, double y, double t) { this.x = x; this.y = y; this.t = t; }
    }

    private List<Intersection> circleSegmentIntersections(double cx, double cy, double r, Waypoint p1, Waypoint p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        double fx = p1.x - cx;
        double fy = p1.y - cy;

        double a = dx*dx + dy*dy;
        double b = 2.0 * (fx*dx + fy*dy);
        double c = (fx*fx + fy*fy) - r*r;

        double disc = b*b - 4*a*c;
        List<Intersection> out = new ArrayList<>();
        if (disc < 0) return out;

        double sqrt = Math.sqrt(disc);
        double t1 = (-b - sqrt) / (2*a);
        double t2 = (-b + sqrt) / (2*a);

        if (t1 >= 0.0 && t1 <= 1.0) out.add(new Intersection(p1.x + t1*dx, p1.y + t1*dy, t1));
        if (t2 >= 0.0 && t2 <= 1.0) out.add(new Intersection(p1.x + t2*dx, p1.y + t2*dy, t2));

        return out;
    }

    private double applyMinPower(double p, double min) {
        if (Math.abs(p) < 1e-6) return 0;
        double sign = Math.signum(p);
        double mag = Math.abs(p);
        if (mag < min) mag = min;
        return sign * mag;
    }

    private double hypot(double a, double b) { return Math.hypot(a, b); }
}