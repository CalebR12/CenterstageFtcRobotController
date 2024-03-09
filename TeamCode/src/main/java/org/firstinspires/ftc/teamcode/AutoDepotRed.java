package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SleeveDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//import static org.firstinspires.ftc.teamcode.Direction.ROBOT_DOWN;


@Autonomous(name="REDDepot", group="CenterStage")
//@Disabled
//@Disabled

public class AutoDepotRed extends AutoBase{
    SleeveDetRedDepot sleeveDetection;
    OpenCvCamera camera;
    @Override
    public void runOpMode() {

        int conePosition = 2;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initHW();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetRedDepot(telemetry);
        camera.setPipeline(sleeveDetection);


        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.rRotate.setPosition(0.75);
        robot.Claw.setPosition(0.25);
        robot.Drone2.setPosition(0);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

     /* \\ while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
           // telemetry.update();
        }*/

        waitForStart();

        telemetry.addData("Path", "Run Complete");
        telemetry.update();

        initMotorEncoders();
        conePosition = sleeveDetection.findConePosition();
        //drivePosition(conePosition);
        //
        //rotate(45,.4);

        if (conePosition == 2) {
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 30, 10.0);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 15, 2);
            myEncoderDrive(Direction.STRAFE_LEFT,DRIVE_SPEED,20,3);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 38, 10.0);
            rotate(-80,TURN_SPEED);
           // sleep(1000);
           // rotate(-45,TURN_SPEED);


        } else if (conePosition==1) {
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 25, 10.0);
            myEncoderDrive(Direction.STRAFE_LEFT,DRIVE_SPEED,9,3);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 10, 2);
            myEncoderDrive(Direction.STRAFE_LEFT,DRIVE_SPEED,10,3);
        }
        else {
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 20, 10.0);
            rotate(-45,TURN_SPEED);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 10, 10.0);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 15, 10.0);
            rotate(45,TURN_SPEED);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 33, 10.0);
            rotate(-80,TURN_SPEED);
            //sleep(1000);
            //rotate(-45,TURN_SPEED);

            // myEncoderDrive(Direction.STRAFE_RIGHT,DRIVE_SPEED,25,3);




        }
    /*public void drivePosition(int conePosition){
        //close and lift cone
        myEncoderLift(Direction.ROBOT_UP, GEAR_SPEED, 8, 1.0);
        //drive to tall pole
        MasterEncoder(Direction.STRAFE_LEFT,Direction.ROBOT_UP, DRIVE_SPEED,GEAR_SPEED, 44, 20, 8);
        myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 4, 2.0);
        robot.lClaw.setPosition(0);
        robot.rClaw.setPosition(0);
        sleep(500);
        myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 4, 2.0);
        MasterEncoder(Direction.STRAFE_LEFT,Direction.ROBOT_DOWN, DRIVE_SPEED,GEAR_SPEED, 18, 14, 4);
        myEncoderLift(Direction.ROBOT_DOWN, GEAR_SPEED, 8, 1.0);
        myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 5, 2.0);
        myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 25.5, 2.0);
        robot.lClaw.setPosition(0.8);
        robot.rClaw.setPosition(0.8);
        sleep(1000);
        myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 1, 1.0);
        myEncoderLift(Direction.ROBOT_UP, GEAR_SPEED, 8, 8.0);
        MasterEncoder(Direction.FORWARD,Direction.ROBOT_UP, DRIVE_SPEED,GEAR_SPEED, 24, 8, 4);
        myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 14, 2.0);
        myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 3, 2.0);
        myEncoderLift(Direction.ROBOT_DOWN, GEAR_SPEED, 4, 1.0);
        robot.rClaw.setPosition(0);
        robot.lClaw.setPosition(0);
        sleep(400);
        myEncoderLift(Direction.ROBOT_UP, GEAR_SPEED, 4, 1.0);
        myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 3, 2.0);
        MasterEncoder(Direction.STRAFE_LEFT,Direction.ROBOT_DOWN, DRIVE_SPEED,GEAR_SPEED, 12, 16, 4);
        myEncoderLift(Direction.ROBOT_DOWN, GEAR_SPEED, 14, 1.0);
        myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 25, 2.0);
        robot.lClaw.setPosition(0.8);
        robot.rClaw.setPosition(0.8);
        sleep(1000);
        myEncoderLift(Direction.ROBOT_UP, GEAR_SPEED, 9, 8.0);
        MasterEncoder(Direction.FORWARD,Direction.ROBOT_UP, DRIVE_SPEED,GEAR_SPEED, 25, 14, 4);
        myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 13, 2.0);
        myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 3, 2.0);
        myEncoderLift(Direction.ROBOT_DOWN, GEAR_SPEED, 5, 1.0);
        robot.rClaw.setPosition(0);
        robot.lClaw.setPosition(0);
        sleep(400);
        myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 3, 2.0);
        myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 13, 2.0);


        if(conePosition == 1){
            myEncoderDrive(Direction.BACKWARD, 0.7, 22, 4.0);
        }else if(conePosition == 3){
            myEncoderDrive(Direction.FORWARD, 0.7, 22, 2.0);
        }
    }*/

    }}
