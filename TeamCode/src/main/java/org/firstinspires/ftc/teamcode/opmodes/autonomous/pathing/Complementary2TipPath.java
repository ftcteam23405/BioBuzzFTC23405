package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Complementary2TipPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(57.5, 132, 90);
    private final Pose leaveEnd = poseFactory.of(58.97, 107.08, 90);
    private final Pose goToFlower1End = poseFactory.of(47.19, 121.35, 90);
    private final Pose goToFlower1Ctrl1 = poseFactory.of(45.25, 110.82, 0);
    private final Pose goAcross1End = poseFactory.of(59.25, 32.79, 90);
    private final Pose goAcross1Ctrl1 = poseFactory.of(68.95, 96.68, 0);
    private final Pose goAcross2End = poseFactory.of(28.2, 9.23, 190);
    private final Pose intakeGardenEnd = poseFactory.of(12.54, 9.51, 190);
    private final Pose goToShoot1End = poseFactory.of(49.68, 9.51, 180);
    private final Pose goToFlower2End = poseFactory.of(18, 47, 180);
    private final Pose goToShoot2End = poseFactory.of(49.68, 9.37, 0);
    private final Pose park1End = poseFactory.of(10.05, 62.04, 0);
    private final Pose park2End = poseFactory.of(9.91, 96.68, 0);

    /** leave — 24.963 in */
    public Path leave() {
        return line(startPose, leaveEnd).constant(90);
    }

    /** goToFlower1 — 20.75 in */
    public Path goToFlower1() {
        return curve(leaveEnd, goToFlower1Ctrl1, goToFlower1End).constant(90);
    }

    /** goAcross1 — 92.003 in */
    public Path goAcross1() {
        return curve(goToFlower1End, goAcross1Ctrl1, goAcross1End).constant(90);
    }

    /** goAcross2 — 38.977 in */
    public Path goAcross2() {
        return line(goAcross1End, goAcross2End).linear(goAcross1End, goAcross2End);
    }

    /** intakeGarden — 15.663 in */
    public Path intakeGarden() {
        return line(goAcross2End, intakeGardenEnd).linear(goAcross2End, intakeGardenEnd);
    }

    /** goToShoot1 — 37.14 in */
    public Path goToShoot1() {
        return line(intakeGardenEnd, goToShoot1End).linear(intakeGardenEnd, goToShoot1End);
    }

    /** goToFlower2 — 49.083 in */
    public Path goToFlower2() {
        return line(goToShoot1End, goToFlower2End).linear(goToShoot1End, goToFlower2End);
    }

    /** goToShoot2 — 49.19 in */
    public Path goToShoot2() {
        return line(goToFlower2End, goToShoot2End).linear(goToFlower2End, goToShoot2End);
    }

    /** park1 — 65.914 in */
    public Path park1() {
        return line(goToShoot2End, park1End).linear(goToShoot2End, park1End);
    }

    /** park2 — 34.64 in */
    public Path park2() {
        return line(park1End, park2End).constant(0);
    }
}
