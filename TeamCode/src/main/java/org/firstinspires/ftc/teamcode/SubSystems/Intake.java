package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Util.Constants;

public class Intake {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    public enum Mode {
        IDLE,
        INTAKE,
        EJECT
    }

    private DcMotorEx roller;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private Mode mode = Mode.IDLE;

    private double scale = 1.0;
    private double targetRPS = Constants.Intake.MAX_INTAKE_RPS;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        roller = hw.get(DcMotorEx.class, Constants.Intake.ROLLER_INTAKE);
        roller.setDirection(DcMotorSimple.Direction.REVERSE);

        roller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setMode(Mode.IDLE);
        apply();
    }

    public void setScale(double s) {
        scale = Range.clip(s, 0.0, 1.0);
    }

    public void setMode(Mode newMode) {
        this.mode = newMode == null ? Mode.IDLE : newMode;
    }

    public Mode getMode() {
        return mode;
    }

    private double intakeRpsToTicksPerSec(double intakeRps) {
        if (roller == null) return 0.0;

        double motorRps = intakeRps * Constants.Intake.MOTOR_PER_INTAKE;
        return motorRps * Constants.Intake.TICKS_PER_REV;
    }

    public double getRps() {
        if (roller == null) return 0.0;
        return (roller.getVelocity() / Constants.Intake.TICKS_PER_REV) / Constants.Intake.MOTOR_PER_INTAKE;
    }

    public void apply() {
        if (roller == null) return;

        switch (mode) {
            case INTAKE: {
                double base = Range.clip(targetRPS, 0.0, Constants.Intake.MAX_INTAKE_RPS);
                double cmdIntakeRps = base * scale;
                roller.setVelocity(intakeRpsToTicksPerSec(cmdIntakeRps));
                break;
            }

            case EJECT: {
                double cmdIntakeRps = Constants.Intake.MAX_INTAKE_RPS * scale;
                roller.setVelocity(-intakeRpsToTicksPerSec(cmdIntakeRps));
                break;
            }

            case IDLE:
            default:
                roller.setVelocity(0.0);
                break;
        }
    }

    public void stop() {
        setMode(Mode.IDLE);
        apply();
    }

    public String getStatus() {
        switch (mode) {
            case INTAKE:
                return "Intake";
            case EJECT:
                return "Eject";
            case IDLE:
            default:
                return "Idle";
        }
    }

    public boolean isActive() {
        return mode != Mode.IDLE;
    }
}
