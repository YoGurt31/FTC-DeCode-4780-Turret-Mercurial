package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Util.Constants;

public class Flywheel {
    public static final Flywheel INSTANCE = new Flywheel();

    private Flywheel() {
    }

    private DcMotorEx fly1, fly2;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private boolean enabled = false;
    private boolean autoRange = true;
    private double manualTargetRps = 0.0;
    private double targetRPS = Double.NaN;

    private double lastF = Double.NaN;
    private double lastVoltage = Double.NaN;

    private static final double VOLTAGE = 0.05; // volts
    private static final double F = 1e-4;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        fly1 = hw.get(DcMotorEx.class, Constants.Flywheel.FLYWHEEL_1);
        fly2 = hw.get(DcMotorEx.class, Constants.Flywheel.FLYWHEEL_2);

        fly1.setDirection(DcMotorEx.Direction.REVERSE);
        fly2.setDirection(DcMotorEx.Direction.FORWARD);

        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        updatePIDF();

        stop();
    }

    private static double rpsToTicksPerSecond(double rps) {
        return rps * Constants.Flywheel.TICKS_PER_REV;
    }

    private void updatePIDF() {
        if (fly1 == null || fly2 == null) return;
        if (Drive.INSTANCE.getBatteryVoltage() <= 1e-6) return;

        boolean voltageChanged = Double.isNaN(lastVoltage) || Math.abs(Drive.INSTANCE.getBatteryVoltage() - lastVoltage) >= VOLTAGE;
        boolean fChanged = Double.isNaN(lastF) || Math.abs(Constants.Flywheel.F(Drive.INSTANCE.getBatteryVoltage()) - lastF) >= F;

        if (voltageChanged || fChanged) {
            fly1.setVelocityPIDFCoefficients(Constants.Flywheel.P, Constants.Flywheel.I, Constants.Flywheel.D, Constants.Flywheel.F(Drive.INSTANCE.getBatteryVoltage()));
            fly2.setVelocityPIDFCoefficients(Constants.Flywheel.P, Constants.Flywheel.I, Constants.Flywheel.D, Constants.Flywheel.F(Drive.INSTANCE.getBatteryVoltage()));
            lastVoltage = Drive.INSTANCE.getBatteryVoltage();
            lastF = Constants.Flywheel.F(Drive.INSTANCE.getBatteryVoltage());
        }
    }

    public void enableAutoRange() {
        autoRange = true;
        enabled = true;
    }

    public void setVelocityRps(double rps) {
        enabled = true;
        autoRange = false;
        manualTargetRps = rps;
    }

    public void stop() {
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.INSTANCE.setVelocityRps(0);
        enabled = false;
        manualTargetRps = 0.0;
        targetRPS = Double.NaN;
        lastF = Double.NaN;
        lastVoltage = Double.NaN;
        if (fly1 != null) fly1.setPower(0);
        if (fly2 != null) fly2.setPower(0);
    }

    public void apply() {
        if (!enabled) return;
        if (fly1 == null || fly2 == null) return;

        updatePIDF();

        double desiredRps = autoRange ? getTargetRps() : manualTargetRps;
        desiredRps = Math.max(Constants.Flywheel.MIN_RPS, Math.min(Constants.Flywheel.MAX_RPS, desiredRps));
        double tps = rpsToTicksPerSecond(desiredRps);

        fly1.setVelocity(tps);
        fly2.setVelocity(tps);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isAutoRange() {
        return autoRange;
    }

    public double getRps1() {
        if (fly1 == null) return 0.0;
        return fly1.getVelocity() / Constants.Flywheel.TICKS_PER_REV;
    }

    public double getRps2() {
        if (fly2 == null) return 0.0;
        return fly2.getVelocity() / Constants.Flywheel.TICKS_PER_REV;
    }

    public double getAverageRps() {
        return (getRps1() + getRps2()) / 2.0;
    }

    public double getTargetRps() {
        // 1) Compute Distance
        double robotX = Drive.INSTANCE.getX();
        double robotY = Drive.INSTANCE.getY();

        Constants.Field.Alliance Alliance = Constants.Field.getAlliance();
        double goalX = Constants.Field.GOAL_X;
        double goalY = (Alliance == Constants.Field.Alliance.RED) ? Constants.Field.RED_GOAL_Y : Constants.Field.BLUE_GOAL_Y;
        double distance = Math.hypot(goalX - robotX, goalY - robotY);

        // 2) Dynamic RPS Model(s)
        double cubicRPS = (Constants.Flywheel.CUBIC_A * Math.pow(distance, 3)) + (Constants.Flywheel.CUBIC_B * Math.pow(distance, 2)) + (Constants.Flywheel.CUBIC_C * distance) + (Constants.Flywheel.CUBIC_D);
        double tableRPS = Double.NaN;
        double[] Xs = Constants.Flywheel.DISTANCE_FROM_TARGET;
        double[] Ys = Constants.Flywheel.RPS;

        if (Xs != null && Ys != null && Xs.length >= 2 && Xs.length == Ys.length) {
            if (distance <= Xs[0]) {
                tableRPS = Ys[0];
            } else if (distance >= Xs[Xs.length - 1]) {
                tableRPS = Ys[Ys.length - 1];
            } else {
                for (int i = 0; i < Xs.length - 1; i++) {
                    double x0 = Xs[i];
                    double x1 = Xs[i + 1];
                    if (distance >= x0 && distance <= x1) {
                        double y0 = Ys[i];
                        double y1 = Ys[i + 1];
                        double t = (distance - x0) / (x1 - x0);
                        tableRPS = y0 + t * (y1 - y0);
                        break;
                    }
                }
            }
        }

        // XXX: Pick & Choose
        double RPS = (0.5 * tableRPS) + (0.5 * cubicRPS);

        // 3) Safety Clamp
        RPS = Math.max(Constants.Flywheel.MIN_RPS, Math.min(Constants.Flywheel.MAX_RPS, RPS));

        // 4) Smooth Target
        double a = Constants.Flywheel.TARGET_SMOOTH_ALPHA;
        if (a < 0.0) a = 0.0;
        if (a > 1.0) a = 1.0;
        if (Double.isNaN(targetRPS)) {
            targetRPS = RPS;
        } else {
            targetRPS = targetRPS + a * (RPS - targetRPS);
        }

        return targetRPS;
    }

    public boolean isReady() {
        double desired = autoRange ? getTargetRps() : manualTargetRps;
        return Math.abs(getAverageRps() - desired) < 2.5;
    }
}
