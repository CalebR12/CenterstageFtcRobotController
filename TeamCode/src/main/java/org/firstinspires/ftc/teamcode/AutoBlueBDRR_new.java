package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name="A_BlueBackDropRoadrun_new", group="ACenterStage")

public class AutoBlueBDRR_new extends LinearOpMode {
    SleeveDetBlueBackDrop sleeveDetection;
    OpenCvCamera camera;

    public AttachHw attach = new AttachHw();
    public ElapsedTime runtime = new ElapsedTime();
    static final double GEAR_PERM = 4.72;
    static final double COUNTS_PER_MOTOR_REV_LIFT = 751.8; //383.6
    static final double COUNTS_PER_INCH_LIFT = COUNTS_PER_MOTOR_REV_LIFT/GEAR_PERM;
    static final double GEAR_SPEED = 0.9;
    double SLIDE_h = 5.0;
    @Override
    public void runOpMode() {
        int conePosition = 2;
        attach.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetBlueBackDrop(telemetry);
        camera.setPipeline(sleeveDetection);
        attach.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        attach.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence BlueCenter= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .splineTo(new Vector2d(32, 0), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(17, 0))
                .lineToLinearHeading(new Pose2d(31, 31, Math.toRadians(90)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 5, 3, true))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.rRotate.setPosition(0.28)) // Lower servo
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.Claw.setPosition(0.25)) // Raise servo
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(32, 28))
                // .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(.65))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_UP", 0.8, SLIDE_h, 3,false))
                .lineToConstantHeading(new Vector2d(4, 0))
                .lineToConstantHeading(new Vector2d(4, -65))
                //.UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rClaw.setPosition(0.35))
                .lineToLinearHeading(new Pose2d(8, -60, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 4, 3,false))
                .lineToLinearHeading(new Pose2d(18.5, -73.5, Math.toRadians(-68)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.Claw.setPosition(0))
                //   .addTemporalMarker(() -> attach.rClaw.setPosition(0.25)) // Raise servo
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(8, -60, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> attach.rRotate.setPosition(0.28))
                //.addTemporalMarker(() -> attach.rRotate.setPosition(0.28))
                .lineToLinearHeading(new Pose2d(4, -65, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(4, 26))
                //    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 3, 3,false))
                // .lineToConstantHeading(new Vector2d(10, -28))
                // .lineToLinearHeading(new Pose2d(10, -26, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(25, -28, Math.toRadians(-90)))
                .addTemporalMarker(28,() -> attach.Claw.setPosition(0.25))


                .build();

        TrajectorySequence BlueRight= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .splineTo(new Vector2d(27, -9), Math.toRadians(-45))
                .lineToConstantHeading(new Vector2d(15, 5))
                .lineToLinearHeading(new Pose2d(38, 31, Math.toRadians(90)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 5, 3, true))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.rRotate.setPosition(0.28)) // Lower servo
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.Claw.setPosition(0.25)) // Raise servo
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(38, 28))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_UP", 0.8, SLIDE_h, 3,false))
                .lineToConstantHeading(new Vector2d(4, 0))
                .lineToConstantHeading(new Vector2d(4, -65))
                //.UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rClaw.setPosition(0.35))
                .lineToLinearHeading(new Pose2d(8, -60, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 4, 3,false))
                .lineToLinearHeading(new Pose2d(18.5, -73.5, Math.toRadians(-68)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.Claw.setPosition(0))
                //   .addTemporalMarker(() -> attach.rClaw.setPosition(0.25)) // Raise servo
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(8, -60, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> attach.rRotate.setPosition(0.28))
                //.addTemporalMarker(() -> attach.rRotate.setPosition(0.28))
                .lineToLinearHeading(new Pose2d(4, -65, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(4, 26))
                //    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 3, 3,false))
                // .lineToConstantHeading(new Vector2d(10, -28))
                // .lineToLinearHeading(new Pose2d(10, -26, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(25, -28, Math.toRadians(-90)))
                .addTemporalMarker(28,() -> attach.Claw.setPosition(0.25))
                .build();

        TrajectorySequence BlueLeft= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .splineTo(new Vector2d(25, 6), Math.toRadians(28))
                .lineToConstantHeading(new Vector2d(12, -1))
                .lineToLinearHeading(new Pose2d(24, 31,Math.toRadians(90)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 5, 3, true))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.rRotate.setPosition(0.28)) // Lower servo
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.Claw.setPosition(0.25)) // Raise servo
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(24, 28))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_UP", 0.8, SLIDE_h, 3,false))
                .lineToConstantHeading(new Vector2d(4, 0))
                .lineToConstantHeading(new Vector2d(4, -65))
             //   .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rClaw.setPosition(0.35))
                .lineToLinearHeading(new Pose2d(8, -60, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 4, 3,false))
                .lineToLinearHeading(new Pose2d(18.5, -73.5, Math.toRadians(-68)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.Claw.setPosition(0))
                //   .addTemporalMarker(() -> attach.rClaw.setPosition(0.25)) // Raise servo
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(8, -60, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> attach.rRotate.setPosition(0.28))
                //.addTemporalMarker(() -> attach.rRotate.setPosition(0.28))
                .lineToLinearHeading(new Pose2d(4, -65, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(4, 26))
                //    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 3, 3,false))
                // .lineToConstantHeading(new Vector2d(10, -28))
                // .lineToLinearHeading(new Pose2d(10, -26, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(25, -28, Math.toRadians(-90)))
                .addTemporalMarker(28,() -> attach.Claw.setPosition(0.25))

                .build();


        attach.rClaw.setPosition(0.35);
        attach.Claw.setPosition(0.0);
        attach.rRotate.setPosition(0.65);
        attach.Drone2.setPosition(0);
        attach.servoArm.setPosition(0.16);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        waitForStart();

        if(isStopRequested()) return;

        conePosition = sleeveDetection.findConePosition();

        if (conePosition == 2) {  //center

            drive.followTrajectorySequence(BlueCenter);
        }
        else if (conePosition ==3 ) //right
        {
            drive.followTrajectorySequence(BlueRight);
        }
        else {
            drive.followTrajectorySequence(BlueLeft);
        }
    }
    public void myEncoderLift(String direction, double speed, double Inches, double timeoutS, boolean calDistance)
    {
        int newLiftTarget = 0;
        int newLift2Target = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        //  liftMotor.setPower(speed);
        //  liftMotor2.setPower(speed);
        //  liftMotor3.setPower(speed);
        if (calDistance) {
            if (attach.sensorD2.getDistance(DistanceUnit.INCH) > 5 &&
                    attach.sensorD2.getDistance(DistanceUnit.INCH) < 10) {
                Inches = attach.sensorD2.getDistance(DistanceUnit.INCH) - 7 + Inches;
                SLIDE_h = Inches;
            }
        }
        //Reset the encoder
        attach.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction.equals("ROBOT_UP")){
                //Go up
                newLiftTarget = attach.liftMotor2.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = attach.liftMotor1.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);

            } else if (direction.equals("ROBOT_DOWN")) {
                //Go down
                newLiftTarget = attach.liftMotor2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = attach.liftMotor1.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH_LIFT);

            } else {
                Inches = 0;
                newLiftTarget = attach.liftMotor2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = attach.liftMotor1.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
            }


            attach.liftMotor1.setTargetPosition(newLiftTarget);
            attach.liftMotor2.setTargetPosition(newLift2Target);



            // Turn On RUN_TO_POSITION
            attach.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            attach.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();

            attach.liftMotor1.setPower(Math.abs(speed));
            attach.liftMotor2.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (attach.liftMotor1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLiftTarget, newLift2Target);
                telemetry.addData("Path2", "Running at %7d :%7d", attach.liftMotor1.getCurrentPosition(),
                        attach.liftMotor2.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            attach.liftMotor1.setPower(0);
            attach.liftMotor2.setPower(0);


            // Turn off RUN_TO_POSITION
            attach.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            attach.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(50);   // optional pause after each move
        }
    }
}


