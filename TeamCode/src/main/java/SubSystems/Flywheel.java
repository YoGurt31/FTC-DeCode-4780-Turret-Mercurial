
package SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

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

    private double lastF = Double.NaN;
    private double lastVoltage = Double.NaN;

    private static final double VOLTAGE = 0.05; // volts
    private static final double F = 1e-4;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        fly1 = hw.get(DcMotorEx.class, Constants.Flywheel.Flywheel1);
        fly2 = hw.get(DcMotorEx.class, Constants.Flywheel.Flywheel2);

        fly1.setDirection(DcMotorEx.Direction.FORWARD);
        fly2.setDirection(DcMotorEx.Direction.REVERSE);

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
        enabled = false;
        manualTargetRps = 0.0;
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

    // TODO: FINALIZE DynamicRPS EQUATION
    public double getTargetRps() {
        // 1) Compute Distance
        double robotX = Drive.INSTANCE.getX();
        double robotY = Drive.INSTANCE.getY();

        int tagId = Vision.INSTANCE.getTrackedTag();
        double goalX = Constants.Field.GOAL_X;
        double goalY = (tagId == Constants.Vision.RED_TAG_ID) ? Constants.Field.RED_GOAL_Y : Constants.Field.BLUE_GOAL_Y;
        double distanceViaPinPoint = Math.hypot(goalX - robotX, goalY - robotY);

        // 2) Camera Range Fusion
        double distance = distanceViaPinPoint;
        if (Vision.INSTANCE.hasTrackedTag()) {
            double distanceViaTurretCam = Vision.INSTANCE.getTrackedRange();
            distance = 0.75 * distanceViaTurretCam + 0.25 * distanceViaPinPoint;
        }

        // 3-1) Linear Model
        double linearRPS = (Constants.Flywheel.M * distance) + Constants.Flywheel.R;

        // 3-2) Sqrt Model
        double sqrtRPS = Math.sqrt(Constants.Flywheel.A * distance + Constants.Flywheel.B) + Constants.Flywheel.C;

        // 4) Safety Clamp
        return Math.max(Constants.Flywheel.MIN_RPS, Math.min(Constants.Flywheel.MAX_RPS, linearRPS));
    }

    public boolean isReady() {
        double desired = autoRange ? getTargetRps() : manualTargetRps;
        return Math.abs(getAverageRps() - desired) < 2.5;
    }
}
