package SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Release {
    public static final Release INSTANCE = new Release();

    private Release() {
    }

    private Servo left;
    private Servo right;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private enum Shot {
        NONE,
        LEFT,
        RIGHT
    }

    private Shot activeShot = Shot.NONE;
    private long shotStartMs = 0;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        left = hw.get(Servo.class, Constants.Releases.leftRelease);
        right = hw.get(Servo.class, Constants.Releases.rightRelease);

        holdBoth();

        activeShot = Shot.NONE;
        shotStartMs = 0;
    }

    public void startLeftShot() {
        if (left == null) return;
        activeShot = Shot.LEFT;
        shotStartMs = System.currentTimeMillis();
    }

    public void startRightShot() {
        if (right == null) return;
        activeShot = Shot.RIGHT;
        shotStartMs = System.currentTimeMillis();
    }

    public void cancel() {
        activeShot = Shot.NONE;
        holdBoth();
    }

    public void update() {
        if (activeShot == Shot.NONE) return;

        long elapsed = System.currentTimeMillis() - shotStartMs;

        boolean gateOpenWindow = elapsed <= Constants.Releases.GATE_OPEN_MS;

        if (activeShot == Shot.LEFT) {
            if (gateOpenWindow) {
                left.setPosition(Constants.Releases.RELEASE_LEFT);
            } else {
                left.setPosition(Constants.Releases.HOLD_LEFT);
            }
        } else if (activeShot == Shot.RIGHT) {
            if (gateOpenWindow) {
                right.setPosition(Constants.Releases.RELEASE_RIGHT);
            } else {
                right.setPosition(Constants.Releases.HOLD_RIGHT);
            }
        }

        if (elapsed >= Constants.Releases.SHOT_TOTAL_MS) {
            if (activeShot == Shot.LEFT) {
                left.setPosition(Constants.Releases.HOLD_LEFT);
            } else if (activeShot == Shot.RIGHT) {
                right.setPosition(Constants.Releases.HOLD_RIGHT);
            }
            activeShot = Shot.NONE;
        }
    }

    public void holdBoth() {
        if (left != null) left.setPosition(Constants.Releases.HOLD_LEFT);
        if (right != null) right.setPosition(Constants.Releases.HOLD_RIGHT);
    }

    public boolean isLeftGateOpen() {
        if (left == null) return false;
        return left.getPosition() == Constants.Releases.RELEASE_LEFT;
    }

    public boolean isRightGateOpen() {
        if (right == null) return false;
        return right.getPosition() == Constants.Releases.RELEASE_RIGHT;
    }

    public boolean isShooting() {
        return activeShot != Shot.NONE;
    }
}
