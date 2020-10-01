package org.firstinspires.ftc.teamcode.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.RevBulkData;

public class ThreeWheelLocalizer {

    private final double TRACK_WIDTH = 0;
    private final double PARALLEL_X_OFFSET = 0;
    private final double PERP_X_OFFSET = 0;
    private final double PERP_Y_OFFSET = 0;
    private final int LEFT_PARALLEL_PORT = 0;
    private final int RIGHT_PARALLEL_PORT = 1;
    private final int PERPENDICULAR_PORT = 2;

    private double TICKS_PER_REV = 8192;
    private double WHEEL_RADIUS = 2.4;

    private double[] prevWheelPositions;
    private DecompositionSolver forwardSolver;
    private PoseExponential poseExponential;

    public Pose2d currentPosition;

    Pose2d[] wheelPoses = {
            new Pose2d(PARALLEL_X_OFFSET, TRACK_WIDTH/2, 0),
            new Pose2d(PARALLEL_X_OFFSET, -TRACK_WIDTH/2, 0),
            new Pose2d(PERP_X_OFFSET, PERP_Y_OFFSET, Math.toRadians(90))
    };

    /**
     * Setup inverse kinematics for wheel positions to inches travelled
     */
    public ThreeWheelLocalizer(Pose2d startPose) {

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3,3);

        //populate a matrix with deadwheel poses
        Vector2d orientationVec;
        Vector2d positionVec;
        for (int i = 0 ; i < 3; i++) {
            orientationVec = wheelPoses[i].headingVec();
            positionVec = wheelPoses[i].vec();
            inverseMatrix.setEntry(i, 0, orientationVec.getX());
            inverseMatrix.setEntry(i, 1, orientationVec.getY());
            inverseMatrix.setEntry(i, 2,
                    positionVec.getX() * orientationVec.getY() - positionVec.getY() * orientationVec.getX());
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }

        //instantiate prevWheelPositions as empty double array with 3 fields
        prevWheelPositions = new double[2];
        //set our currentposition to the start pose declared in the opmode
        currentPosition = new Pose2d(startPose.getX(), startPose.getY(), startPose.getTheta());
    }

    public double encoderTicksToCM(double ticks) {
        return 2 * Math.PI * WHEEL_RADIUS * ticks / TICKS_PER_REV;
    }

    /*
    Calculate the robot pose x, y, and heading changes from the wheel changes
     */
    public Pose2d calculatePoseDeltas(double[] deltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());

        return new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    /*
    Update the robots pose on the field
     */
    public void update(RevBulkData data) {
        double leftTicks = data.getMotorCurrentPosition(LEFT_PARALLEL_PORT);
        double rightTicks = data.getMotorCurrentPosition(RIGHT_PARALLEL_PORT);
        double perpTicks = data.getMotorCurrentPosition(PERPENDICULAR_PORT);

        //get the wheel changes this loop (in CM)
        double[] wheelDeltas = new double[] {
                encoderTicksToCM(leftTicks - prevWheelPositions[0]),
                encoderTicksToCM(rightTicks - prevWheelPositions[1]),
                encoderTicksToCM(perpTicks - prevWheelPositions[2])
        };

        //Calculate the robot pose change from the wheel deltas
        Pose2d robotPoseDeltas = calculatePoseDeltas(wheelDeltas);
        //update current pose through pose exponential: add global pose change to previous global pose
        currentPosition = poseExponential.globalOdometryUpdate(currentPosition, robotPoseDeltas);
        //store wheel positions for changes next loop
        prevWheelPositions[0] = leftTicks;
        prevWheelPositions[1] = rightTicks;
        prevWheelPositions[2] = perpTicks;
    }

    public Pose2d getGlobalPose() {
        return currentPosition;
    }

}
