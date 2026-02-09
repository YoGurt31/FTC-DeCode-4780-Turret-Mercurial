package SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Release {
    public static final Release INSTANCE = new Release();

    private Release() {
    }

    private Servo release;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private enum Shot {
        NONE,
        SHOOTING
    }

    private Shot activeShot = Shot.NONE;
    private long shotStartMs = 0;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        release = hw.get(Servo.class, Constants.Releases.artifactRelease);

        hold();

        activeShot = Shot.NONE;
        shotStartMs = 0;
    }

    public void startShot() {
        if (release == null) return;
        activeShot = Shot.SHOOTING;
        shotStartMs = System.currentTimeMillis();
    }

    public void cancel() {
        activeShot = Shot.NONE;
        hold();
    }

    public void update() {
        if (activeShot == Shot.NONE) return;

        long elapsed = System.currentTimeMillis() - shotStartMs;

        boolean gateOpenWindow = elapsed <= Constants.Releases.GATE_OPEN_MS;

        if (activeShot == Shot.SHOOTING) {
            if (gateOpenWindow) {
                release.setPosition(Constants.Releases.RELEASE);
            } else {
                release.setPosition(Constants.Releases.HOLD);
            }
        }

        if (elapsed >= Constants.Releases.SHOT_TOTAL_MS) {
            if (activeShot == Shot.SHOOTING) {
                release.setPosition(Constants.Releases.HOLD);
            }
            activeShot = Shot.NONE;
        }
    }

    public void hold() {
        if (release != null) release.setPosition(Constants.Releases.HOLD);
    }

    public boolean isGateOpen() {
        if (release == null) return false;
        return release.getPosition() == Constants.Releases.RELEASE;
    }

    public boolean isShooting() {
        return activeShot != Shot.NONE;
    }
}
