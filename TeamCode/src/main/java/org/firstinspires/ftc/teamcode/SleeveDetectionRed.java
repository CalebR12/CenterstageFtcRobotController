package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetectionRed extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    //private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);
    private static Point LEFT_ANCHOR_POINT = new Point(109, 98);
    static final Point MIDDLE_ANCHOR_POINT = new Point(181,98);
    static final Point RIGHT_ANCHOR_POINT = new Point(253,98);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 20;
    public static int REGION_HEIGHT = 20;

    // Color definitions
    private final Scalar
            RED  = new Scalar(255, 0, 0),
            BLUE    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Anchor point definitions
    Point left_pointA = new Point(
            LEFT_ANCHOR_POINT.x,
            LEFT_ANCHOR_POINT.y);
    Point left_pointB = new Point(
            LEFT_ANCHOR_POINT.x + REGION_WIDTH,
            LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point middle_pointA = new Point(
            MIDDLE_ANCHOR_POINT.x,
            MIDDLE_ANCHOR_POINT.y);
    Point middle_pointB = new Point(
            MIDDLE_ANCHOR_POINT.x + REGION_WIDTH,
            MIDDLE_ANCHOR_POINT.y + REGION_HEIGHT);


    Point right_pointA = new Point(
            RIGHT_ANCHOR_POINT.x,
            RIGHT_ANCHOR_POINT.y);
    Point right_pointB = new Point(
            RIGHT_ANCHOR_POINT.x + REGION_WIDTH,
            RIGHT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;
    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.RIGHT;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(left_pointA, left_pointB));
        region2_Cb = Cb.submat(new Rect(middle_pointA, middle_pointB));
        region3_Cb = Cb.submat(new Rect(right_pointA, right_pointB));
    }



    @Override
    public Mat processFrame(Mat input) {
        // Get the submat frame, and then sum all the values
        //   Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColorsL = Core.sumElems(region1_Cb);
        Scalar sumColorsM = Core.sumElems(region2_Cb);
        Scalar sumColorsR = Core.sumElems(region3_Cb);


        // Get the minimum RGB value from every single channel
        double minColorL = Math.min(sumColorsL.val[0], Math.min(sumColorsL.val[1], sumColorsL.val[2]));
        double minColorM = Math.min(sumColorsM.val[0], Math.min(sumColorsM.val[1], sumColorsM.val[2]));
        double minColorR = Math.min(sumColorsR.val[0], Math.min(sumColorsR.val[1], sumColorsR.val[2]));

        // Change the bounding box color based on the sleeve color
        if (sumColorsL.val[1] == minColorL && sumColorsL.val[2] ==minColorL && sumColorsL.val[0]>225) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    left_pointA,
                    left_pointB,
                    RED,
                    2
            );
        } else if (sumColorsM.val[1] == minColorM && sumColorsM.val[2] ==minColorM && sumColorsM.val[0]>225) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    middle_pointB,
                    middle_pointA,
                    RED,
                    2
            );
        } else {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    right_pointA,
                    right_pointB,
                    RED,
                    2
            );
        }

        // Release and return input
        //   areaMat.release();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public String getPosition() {
        return position.toString();
    }
}