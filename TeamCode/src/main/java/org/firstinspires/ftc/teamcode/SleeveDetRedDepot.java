package org.firstinspires.ftc.teamcode;

import android.location.Location;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.channels.ClosedChannelException;

public class SleeveDetRedDepot extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public SleeveDetRedDepot( Telemetry t){
        telemetry = t;
    }
    static final Rect CENTER_ROI = new Rect(new Point(165, 55), new Point(195, 85));
    static final Rect LEFT_ROI = new Rect(new Point(25, 55), new Point(55,85));
    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    String ColorV = "";
    private volatile ParkingPosition position = ParkingPosition.RIGHT;

    // Color definitions
    private final Scalar
            RED  = new Scalar(255, 0, 0),
            BLUE    = new Scalar(0, 0, 255),
            MAGENTA = new Scalar(255, 0, 255);
    static double PER_COL_TH = 0.4;



    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHsv = new Scalar(155,100,100);
        Scalar highHsv = new Scalar(179,255,255);
        Core.inRange(mat, lowHsv,highHsv,mat);

        Mat left = mat.submat(CENTER_ROI);
        Mat right = mat.submat(LEFT_ROI);


        double leftValue = Core.sumElems(left).val[0]/CENTER_ROI.area()/255;
        double rightValue = Core.sumElems(right).val[0]/LEFT_ROI.area()/255;

        left.release();
        right.release();

        ////
        Mat areaMat = input.submat(CENTER_ROI);
        Scalar sumColors = Core.sumElems(areaMat);
        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));


        telemetry.addData("CENTER ","Min color " + String.valueOf(minColor) + " val 0 " +
                String.valueOf(sumColors.val[0]) + " value1 "
                + String.valueOf(sumColors.val[1]) + " val 2 "
                + String.valueOf(sumColors.val[2]));

        Mat areaMatR = input.submat(LEFT_ROI);
        Scalar sumColorsR = Core.sumElems(areaMatR);
        double minColorR = Math.min(sumColorsR.val[0], Math.min(sumColorsR.val[1], sumColorsR.val[2]));


        telemetry.addData("RIGHT ","Min color " + String.valueOf(minColorR) + " val 0 " +
                String.valueOf(sumColorsR.val[0]) + " value1 "
                + String.valueOf(sumColorsR.val[1]) + " val 2 "
                + String.valueOf(sumColorsR.val[2]));

        // Get the minimum RGB value from every single channel

       /* telemetry.addData("Left value ",(int)Core.sumElems(left).val[0]);
        telemetry.addData("Right value ",(int)Core.sumElems(right).val[0]);
        telemetry.addData("Left 2 value ",(int)Core.sumElems(left).val[2]);
        telemetry.addData("Right 2 value ",(int)Core.sumElems(right).val[2]);
        telemetry.addData("Left ori value ",leftValue);
        telemetry.addData("Right ori value ",rightValue);
        telemetry.addData("Left percent ",Math.round(leftValue*100)+ "%");
        telemetry.addData("Right percent ",Math.round(rightValue*100)+ "%");*/

        boolean stonecenter = ((sumColors.val[0] > sumColors.val[1]) && (sumColors.val[0] > sumColors.val[2]));
        boolean stoneright = ((sumColorsR.val[0] > sumColorsR.val[1]) && (sumColorsR.val[0] > sumColorsR.val[2]));

       /* if (stoneleft && stoneright) {
            position = ParkingPosition.RIGHT;
            telemetry.addData("Location is ", "RIGHT");
        } */
        if (stonecenter) {
            position = ParkingPosition.CENTER;
            telemetry.addData("Location is ", "CENTER");
        }
        else if (stoneright) {
            position = ParkingPosition.LEFT;
            telemetry.addData("Location is ", "LEFT");
        }
        else {
            position = ParkingPosition.RIGHT;
            telemetry.addData("Location is ", "RIGHT");
        }
        telemetry.update();

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Scalar colorstone = new Scalar(255,0,0);
        Scalar foundstone = new Scalar(0,255,0);

        Imgproc.rectangle(input,CENTER_ROI, position == ParkingPosition.CENTER? foundstone:colorstone );
        Imgproc.rectangle(input,LEFT_ROI, position == ParkingPosition.RIGHT? foundstone:colorstone );

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public String getPosition() {
        return position.toString();
        // return ColorV;
    }


    public int findConePosition (){
        int pos = 3;
        if (getPosition() == "LEFT"){
            pos = 1;
        } else if (getPosition()== "CENTER"){
            pos = 2;
        }
        return pos;
    }
}