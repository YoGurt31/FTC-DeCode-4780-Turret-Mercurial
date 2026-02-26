package OpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoadRunner.TankDrive;
import Util.Constants;

@Autonomous(name = "Blue Close")
public final class BlueClose extends Auton.BaseAuto {
    @Override
    protected Constants.Field.StartPose startPoseDefined() {
        return Constants.Field.StartPose.BLUE_CLOSE;
    }

    @Override
    protected Constants.Field.Alliance alliance() {
        return Constants.Field.Alliance.BLUE;
    }

    @Override
    protected Action buildMain(TankDrive drive, Pose2d startPose, boolean[] intakeEnabled) {
        return Auton.Paths.blueClose(drive, startPose, intakeEnabled);
    }
}
