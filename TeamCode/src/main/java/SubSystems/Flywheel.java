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

        double desiredRps = 80;
//        double desiredRps = getTargetRps();
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

    // TODO: CREATE EQUATION FOR VARIABLE RPS
    public double getTargetRps() {
        // 1) Compute Distance
        double robotX = Drive.INSTANCE.getX();
        double robotY = Drive.INSTANCE.getY();

        int tagId = Vision.INSTANCE.getTrackedTag();
        double goalX = Constants.Field.GOAL_X;
        double goalY = (tagId == Constants.Vision.RED_TAG_ID)
                ? Constants.Field.RED_GOAL_Y
                : Constants.Field.BLUE_GOAL_Y;

        double distanceViaPinPoint = Math.hypot(goalX - robotX, goalY - robotY);

        // 2) Camera Range Fusion
        double distance = distanceViaPinPoint;
        if (Vision.INSTANCE.hasTrackedTag()) {
            double distanceViaTurretCam = Vision.INSTANCE.getTrackedRange();
            distance = 0.6 * distanceViaTurretCam + 0.4 * distanceViaPinPoint;
        }

        // 3) Sqrt Model
        double rps = Math.sqrt(Constants.Flywheel.A * distance + Constants.Flywheel.B) + Constants.Flywheel.C;

        // 4) Safety Clamp
        rps = Math.max(Constants.Flywheel.MIN_RPS, Math.min(Constants.Flywheel.MAX_RPS, rps));
        return rps;
    }

    public boolean isReady() {
        double desired = getTargetRps();
        return Math.abs(getAverageRps() - desired) < 0.5;
    }
}
