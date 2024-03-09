package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="AUtoTestSLide", group="BCenterStage")

public class AutoSlideTest extends LinearOpMode {
    SleeveDetRedDepot sleeveDetection;
    OpenCvCamera camera;

    public AttachHw attach = new AttachHw();
    public ElapsedTime runtime = new ElapsedTime();
    static final double GEAR_PERM = 4.72;
    static final double COUNTS_PER_MOTOR_REV_LIFT = 751.8; //383.6
    static final double COUNTS_PER_INCH_LIFT = COUNTS_PER_MOTOR_REV_LIFT/GEAR_PERM;
    static final double GEAR_SPEED = 0.9;

    public void myEncoderLift(String direction, double speed, double Inches, double timeoutS)
    {
        int newLiftTarget = 0;
        int newLift2Target = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        //  liftMotor.setPower(speed);
        //  liftMotor2.setPower(speed);
        //  liftMotor3.setPower(speed);

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

    @Override
    public void runOpMode() {
        int conePosition = 2;
        attach.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetRedDepot(telemetry);
        camera.setPipeline(sleeveDetection);
        attach.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        attach.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attach.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence RedCenter= drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0.00)))
                .addTemporalMarker(() -> myEncoderLift("ROBOT_DOWN", 0.8, 7, 3))
                .waitSeconds(2)
                .addTemporalMarker(() -> myEncoderLift("ROBOT_UP", 0.8, 7, 3))

                .build();
        attach.rClaw.setPosition(0.25);
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

           drive.followTrajectorySequence(RedCenter);


        }

    }
