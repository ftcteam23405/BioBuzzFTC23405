package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class CloseStart1TipParkPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56, 9, 270);
    private final Pose shoot1End = poseFactory.of(56, 27, 270);
    private final Pose parkPose = poseFactory.of(13.81, 93.04, 270);
    private final Pose parkCtrl1 = poseFactory.of(20.02, 32.89, 0);

    /** New move — 18 in */
    public Path newMove() {
        return line(startPose, shoot1End).constant(270);
    }

    /** New move — 84.616 in */
    public Path newMove2() {
        return curve(shoot1End, parkCtrl1, parkPose).linear(shoot1End, parkPose);
    }
}
