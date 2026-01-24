package SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Intake {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    public enum Mode {
        IDLE,
        LEFT,
        RIGHT,
        EJECT
    }

    private DcMotorEx roller, sorter;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private Mode mode = Mode.IDLE;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        roller = hw.get(DcMotorEx.class, Constants.Intake.rollerIntake);
        sorter = hw.get(DcMotorEx.class, Constants.Intake.sorterIntake);

        roller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setMode(Mode.IDLE);
        apply();
    }

    public void setMode(Mode newMode) {
        this.mode = newMode == null ? Mode.IDLE : newMode;
    }

    public Mode getMode() {
        return mode;
    }

    public void apply() {
        if (roller == null || sorter == null) return;

        switch (mode) {
            case LEFT:
                roller.setPower(Constants.Intake.INTAKE_POWER);
                sorter.setPower(Constants.Intake.INTAKE_POWER);
                break;

            case RIGHT:
                roller.setPower(Constants.Intake.INTAKE_POWER);
                sorter.setPower(-Constants.Intake.INTAKE_POWER);
                break;

            case EJECT:
                roller.setPower(Constants.Intake.OUTTAKE_POWER);
                sorter.setPower(0);
                break;

            case IDLE:
            default:
                roller.setPower(0);
                sorter.setPower(0);
                break;
        }
    }

    public void stop() {
        setMode(Mode.IDLE);
        apply();
    }

    public String getSortingStatus() {
        switch (mode) {
            case LEFT:
                return "Left";
            case RIGHT:
                return "Right";
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
