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
    private double manualTargetRps = Constants.Flywheel.FAR_TARGET_RPS;

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

        fly1.setVelocityPIDFCoefficients(Constants.Flywheel.P, Constants.Flywheel.I, Constants.Flywheel.D, Constants.Flywheel.F);
        fly2.setVelocityPIDFCoefficients(Constants.Flywheel.P, Constants.Flywheel.I, Constants.Flywheel.D, Constants.Flywheel.F);

        stop();
    }

    private static double rpsToTicksPerSecond(double rps) {
        return rps * Constants.Flywheel.TICKS_PER_REV;
    }

    public void setTargetRps(double rps) {
        manualTargetRps = rps;
        autoRange = false;
        enabled = true;
    }

    public void enableAutoRange() {
        autoRange = true;
        enabled = true;
    }

    public void stop() {
        enabled = false;
        if (fly1 != null) fly1.setPower(0);
        if (fly2 != null) fly2.setPower(0);
    }

    public void apply() {
        if (!enabled) return;
        if (fly1 == null || fly2 == null) return;

        double desiredRps = autoRange ? getTargetRps() : manualTargetRps;
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

    public double getManualTargetRps() {
        return manualTargetRps;
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

    // TODO: CREATE EQUATION FOR VARIABLE RPS
    public double getTargetRps() {
//        if (Vision.INSTANCE.getTA() >= Constants.Vision.TAG_AREA_THRESHOLD) {
//            return Constants.Flywheel.CLOSE_TARGET_RPS;
//        } else {
            return Constants.Flywheel.FAR_TARGET_RPS;
//        }
    }

    public boolean rangeMode() {
        return getTargetRps() < Constants.Flywheel.FAR_TARGET_RPS;
    }

    public boolean isReady() {
        double desired = autoRange ? getTargetRps() : manualTargetRps;
        return Math.abs(getAverageRps() - desired) < 0.5;
    }
}
