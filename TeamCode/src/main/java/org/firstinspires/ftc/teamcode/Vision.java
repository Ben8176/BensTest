package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Vision
{
    HardwareMap hardwareMap;
    double area = -1;

    //Outputs Mats
    private Mat hsvThresholdOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    private Mat cvDilateOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();

    //Constructor that brings in the hardwareMap from the robot hardware class
    public Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    OpenCvCamera webcam;

    /**
     * Get the camera initialized and set the pipeline. And start the camera streaming
     */
    public void initVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(new RingDetector());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public void stopStreaming() {
        webcam.stopStreaming();
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class RingDetector extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        Mat hsvThresholdInput;
        double[] hsvThresholdHue = {8.093525179856115, 21.808873720136525};
        double[] hsvThresholdSaturation = {114.65827338129498, 255.0};
        double[] hsvThresholdValue = {105.94424460431654, 255.0};

        Mat cvErodeSrc;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 9.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);

        Mat cvDilateSrc;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 11.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);

        Mat findContoursInput;
        boolean findContoursExternalOnly = false;

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */
            hsvThresholdInput = input;

            //Do the hsv threshold operation on the image
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
            cvErodeSrc = hsvThresholdOutput;

            //do the erode operation on the hsv image
            cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);
            cvDilateSrc = cvErodeOutput;

            //cvDilate operation on the cv eroded output
            cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);
            findContoursInput = cvDilateOutput;

            //find contours of the shape operation
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

            //-----------------------------------------------------------------------------------

            /**
             * Our magic variable update goes here
             * Get the area of the rectangle (contour) that we found above
             */
            area = Imgproc.contourArea(findContoursOutput().get(0));

            //-----------------------------------------------------------------------------------


            return input;
        }

        /*
        Get the rectangle area from the image process method
         */
        public double getArea() {
            return area;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }

        /**
         * This method is a generated getter for the output of a HSV_Threshold.
         * @return Mat output from HSV_Threshold.
         */
        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a CV_erode.
         * @return Mat output from CV_erode.
         */
        public Mat cvErodeOutput() {
            return cvErodeOutput;
        }

        /**
         * This method is a generated getter for the output of a CV_dilate.
         * @return Mat output from CV_dilate.
         */
        public Mat cvDilateOutput() {
            return cvDilateOutput;
        }

        /**
         * This method is a generated getter for the output of a Find_Contours.
         * @return ArrayList<MatOfPoint> output from Find_Contours.
         */
        public ArrayList<MatOfPoint> findContoursOutput() {
            return findContoursOutput;
        }

        /**
         * Segment an image based on hue, saturation, and value ranges.
         *
         * @param input The image on which to perform the HSL threshold.
         * @param hue The min and max hue
         * @param sat The min and max saturation
         * @param val The min and max value
         * @param out The image in which to store the output.
         */
        private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
            Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                    new Scalar(hue[1], sat[1], val[1]), out);
        }

        /**
         * Expands area of lower value in an image.
         * @param src the Image to erode.
         * @param kernel the kernel for erosion.
         * @param anchor the center of the kernel.
         * @param iterations the number of times to perform the erosion.
         * @param borderType pixel extrapolation method.
         * @param borderValue value to be used for a constant border.
         * @param dst Output Image.
         */
        private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                             int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1,-1);
            }
            if (borderValue == null) {
                borderValue = new Scalar(-1);
            }
            Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
        }

        /**
         * Expands area of higher value in an image.
         * @param src the Image to dilate.
         * @param kernel the kernel for dilation.
         * @param anchor the center of the kernel.
         * @param iterations the number of times to perform the dilation.
         * @param borderType pixel extrapolation method.
         * @param borderValue value to be used for a constant border.
         * @param dst Output Image.
         */
        private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                              int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1,-1);
            }
            if (borderValue == null){
                borderValue = new Scalar(-1);
            }
            Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
        }

        /**
         * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
         */
        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }
    }
}