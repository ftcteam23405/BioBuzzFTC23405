package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class CloseStart2TipPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(54, 9, 90);
    private final Pose intakeEnd = poseFactory.of(11.56, 11.05, 240);
    private final Pose intakeCtrl1 = poseFactory.of(40.26, 27.29, 0);
    private final Pose goAcross1End = poseFactory.of(58, 26, 90);
    private final Pose goAcross2End = poseFactory.of(58, 117, 90);
    private final Pose goToFlowerEnd = poseFactory.of(47, 123, 90);
    private final Pose parkEnd = poseFactory.of(16, 117, 0);

    /** intake — 47.108 in */
    public Path intake() {
        return curve(startPose, intakeCtrl1, intakeEnd).linear(startPose, intakeEnd);
    }

    /** goAcross1 — 48.787 in */
    public Path goAcross1() {
        return line(intakeEnd, goAcross1End).linear(intakeEnd, goAcross1End);
    }

    /** goAcross2 — 91 in */
    public Path goAcross2() {
        return line(goAcross1End, goAcross2End).constant(90);
    }

    /** goToFlower — 12.53 in */
    public Path goToFlower() {
        return line(goAcross2End, goToFlowerEnd).linear(goAcross2End, goToFlowerEnd);
    }

    /** park — 31.575 in */
    public Path park() {
        return line(goToFlowerEnd, parkEnd).linear(goToFlowerEnd, parkEnd);
    }
}
