package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;

public class PoseExponential {

    MathUtil m;
    public PoseExponential() {

    }

    public Pose2d globalOdometryUpdate(Pose2d lastPose, Pose2d poseDeltas) {
        //get delta theta from robot pose deltas
        double dtheta = poseDeltas.getTheta();

        //if dtheta is small enough, certain equations model it better
        double sinTerm = m.epsilonEquals(dtheta, 0) ? 1 - ((dtheta*dtheta) / 6.0) : Math.sin(dtheta) / dtheta;
        double cosTerm = m.epsilonEquals(dtheta, 0) ? dtheta / 2.0 : (1 - Math.cos(dtheta)) / dtheta;

        Vector2d fieldPositionDelta = new Vector2d(
                sinTerm * poseDeltas.getX() - cosTerm * poseDeltas.getY(),
                cosTerm * poseDeltas.getX() + sinTerm * poseDeltas.getY()
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotateBy(lastPose.getTheta()), poseDeltas.getTheta());

        return new Pose2d(
                lastPose.getX() + fieldPoseDelta.getX(),
                lastPose.getY() + fieldPoseDelta.getY(),
                //make sure to angle wrap this number so it doesnt get above 2pi
                m.wrapToTau(lastPose.getTheta() + fieldPoseDelta.getTheta())
        );
    }
}
