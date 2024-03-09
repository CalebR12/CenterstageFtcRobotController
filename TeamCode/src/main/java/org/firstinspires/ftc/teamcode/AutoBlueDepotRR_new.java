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
@Autonomous(name="A_BlueDepotRR_new", group="ACenterStage")

public class AutoBlueDepotRR_new extends LinearOpMode {
    SleeveDetectionBlue sleeveDetection;
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
        sleeveDetection = new SleeveDetectionBlue(telemetry);
        camera.setPipeline(sleeveDetection);
        attach.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        attach.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence BlueCenter= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .splineTo(new Vector2d(32, 0), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(32, 4))
                .lineToConstantHeading(new Vector2d(17, 4))
                .lineToConstantHeading(new Vector2d(17, -5))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 5.05, 3,false))
                .lineToLinearHeading(new Pose2d(50,-13,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(50, -15))
                .waitSeconds(.2)
                .addTemporalMarker(() -> attach.Claw.setPosition(0)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(50, -10))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0.20))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> myEncoderLift("ROBOT_UP", 0.8, 5.1, 3,false))
                .lineToConstantHeading(new Vector2d(50, 72))  //-72
                .lineToLinearHeading(new Pose2d(40,75,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0.65))
                .lineToLinearHeading(new Pose2d(24,81,Math.toRadians(90)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 8, 3, true))
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(24, 83),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> attach.rRotate.setPosition(0.28)) // Lower servo
                .waitSeconds(0.4)
                .addTemporalMarker(() -> attach.rClaw.setPosition(0.0)) // Raise servo
                .waitSeconds(0.3)
                .addTemporalMarker(() -> attach.Claw.setPosition(0.28)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(24, 82),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_UP", 0.8, 8, 3,false))
                .lineToConstantHeading(new Vector2d(48, 82))
                .build();

        TrajectorySequence BlueLeft= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .splineTo(new Vector2d(27, 9), Math.toRadians(45))
                .lineToConstantHeading(new Vector2d(15, -5))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 5.05, 3,false))
                .lineToLinearHeading(new Pose2d(50,-13,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(50, -15))
                .waitSeconds(.2)
                .addTemporalMarker(() -> attach.Claw.setPosition(0)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(50, 10))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0.20))
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> myEncoderLift("ROBOT_UP", 0.8, 5.05, 3,false))
      //          .addTemporalMarker(() -> attach.rRotate.setPosition(.20))
        //        .addTemporalMarker(() -> myEncoderLift("ROBOT_UP", 0.8, 5.05, 3, false))
//                .lineToConstantHeading(new Vector2d(50, 70))  //-72
//                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(50, 72))  //-72
                .lineToLinearHeading(new Pose2d(40,75,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0.65))
                .lineToLinearHeading(new Pose2d(31,80,Math.toRadians(90)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 8, 3, true))
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(31, 82),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> attach.rRotate.setPosition(0.28)) // Lower servo
                .waitSeconds(0.4)
                .addTemporalMarker(() -> attach.Claw.setPosition(0.25)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(31, 75))
                .lineToConstantHeading(new Vector2d(18, 75))
                .lineToConstantHeading(new Vector2d(18, 82),  SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> attach.rClaw.setPosition(0)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(18, 81))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_UP", 0.8, 8, 3,false))
                //   .addTemporalMarker(() -> myEncoderLift("ROBOT_UP", 0.8, 8, 3, true))
                .lineToConstantHeading(new Vector2d(48, 81))
                .build();

        TrajectorySequence BlueRight= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .splineTo(new Vector2d(25, -6), Math.toRadians(-28))
                .lineToConstantHeading(new Vector2d(12, 1))
                .lineToLinearHeading(new Pose2d(20,2,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(38, 2))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_DOWN", 0.8, 5.05, 3,false))
                .lineToLinearHeading(new Pose2d(50,-13,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(50, -15))
                .waitSeconds(.2)
                .addTemporalMarker(() -> attach.Claw.setPosition(0)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(50, -10))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0.20))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> myEncoderLift("ROBOT_UP", 0.8, 5.1, 3,false))
                .lineToConstantHeading(new Vector2d(50, 72))  //-72
                .lineToLinearHeading(new Pose2d(40,75,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> attach.rRotate.setPosition(0.65))
                .lineToLinearHeading(new Pose2d(30,81,Math.toRadians(90)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 8, 3, true))
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(30, 82),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> attach.rRotate.setPosition(0.28)) // Lower servo
                .waitSeconds(0.4)
                .addTemporalMarker(() -> attach.rClaw.setPosition(0.0)) // Raise servo
                .addTemporalMarker(() -> attach.Claw.setPosition(0.25)) // Raise servo
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(30, 81),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> myEncoderLift("ROBOT_UP", 0.8, 8, 3,false))
                .lineToConstantHeading(new Vector2d(48, 81))

                .build();

        attach.rClaw.setPosition(0.25);
        attach.Claw.setPosition(0.25);
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

