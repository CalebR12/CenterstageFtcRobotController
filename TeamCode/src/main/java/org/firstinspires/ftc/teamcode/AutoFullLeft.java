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


@Autonomous(name="Lsmall", group="CenterStage")
//@Disabled
//@Disabled

public class AutoFullLeft extends AutoBase{
    SleeveDetectionLeft sleeveDetection;
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
        sleeveDetection = new SleeveDetectionLeft();
        camera.setPipeline(sleeveDetection);


        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.lClaw.setPosition(0.4);
        robot.rClaw.setPosition(0.0);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();

        telemetry.addData("Path", "Run Complete");
        telemetry.update();

        initMotorEncoders();
        conePosition = findConePosition(sleeveDetection.getPosition());
        //drivePosition(conePosition);
        //    myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 20, 10.0);
        //  rotate(36, TURN_SPEED);
        //  myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 19, 10.0);
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
    private int findConePosition (String pos){
        int position = 3;
        if (pos == "LEFT"){
            position = 1;
        } else if (pos == "CENTER"){
            position = 2;
        }
        return position;
    }
}
