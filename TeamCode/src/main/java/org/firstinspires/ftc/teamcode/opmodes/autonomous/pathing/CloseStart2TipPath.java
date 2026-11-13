package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class CloseStart2TipPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56.5, 7.5, 180);
    private final Pose intakeEnd = poseFactory.of(12, 9, 190);
    private final Pose intakeCtrl1 = poseFactory.of(35.96, 14.5, 0);
    private final Pose goAcross1End = poseFactory.of(63.4, 25.59, 90);
    private final Pose goAcross2End = poseFactory.of(63, 116, 90);
    private final Pose goToFlowerEnd = poseFactory.of(47, 123, 90);
    private final Pose parkEnd = poseFactory.of(9.77, 116.36, 0);

    /** intake — 45.115 in */
    public Path intake() {
        return curve(startPose, intakeCtrl1, intakeEnd).linear(startPose, intakeEnd);
    }

    /** goAcross1 — 54.011 in */
    public Path goAcross1() {
        return line(intakeEnd, goAcross1End).linear(intakeEnd, goAcross1End);
    }

    /** goAcross2 — 90.411 in */
    public Path goAcross2() {
        return line(goAcross1End, goAcross2End).constant(90);
    }

    /** goToFlower — 17.464 in */
    public Path goToFlower() {
        return line(goAcross2End, goToFlowerEnd).linear(goAcross2End, goToFlowerEnd);
    }

    /** park — 37.817 in */
    public Path park() {
        return line(goToFlowerEnd, parkEnd).linear(goToFlowerEnd, parkEnd);
    }
}