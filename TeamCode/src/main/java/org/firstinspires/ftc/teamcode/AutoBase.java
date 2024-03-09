package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

enum Direction
{
    FORWARD, BACKWARD, STRAFE_RIGHT, STRAFE_LEFT, ROBOT_UP, ROBOT_DOWN
}


public class AutoBase extends LinearOpMode
{
    public MasterHW robot = new MasterHW();
    public ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0;
    public Direction direction;
    double ref_angle = 0;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

       static final double WHEEL_PERM = 11.873743682;
    static final double COUNTS_PER_MOTOR_REV = 537.7; //383.6
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV/WHEEL_PERM;

    static final double GEAR_PERM = 6.2;
    static final double COUNTS_PER_MOTOR_REV_LIFT = 752.8; //383.6
    static final double COUNTS_PER_INCH_LIFT = COUNTS_PER_MOTOR_REV_LIFT/GEAR_PERM;

    static final double DRIVE_SPEED = 0.5;
    static final double GEAR_SPEED = 0.9;
    static final double TURN_SPEED = 0.5;

    public TFObjectDetector tfod = null;
   // public VuforiaLocalizer vuforia = null;



    @Override
    public void runOpMode()
    {
        //Empty Function
    }
    public void initHW1(HardwareMap hardwareMap) {
        RobotLog.ii("CAL", "Enter -  initHW");
        robot.init(hardwareMap);
    }
    public void initHW() {
        RobotLog.ii("CAL", "Enter -  initHW");
        robot.init(hardwareMap);
        //initMotorEncoders();
        //       initVuforia();
        //     initTfod();

     /*   if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

      */

    }

    public void initMotorEncoders()
    {
        RobotLog.ii("CAL", "Enter -  initMotorEncoders");
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // robot.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // robot.liftMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //robot.MLanderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //robot.MLanderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.ii("CAL", "Exit -  initMotorEncoders");

        telemetry.addData("Path1", "Init MotorEncoders Done");
        telemetry.update();
    }
    private void resetAngle()
    {
        lastAngles = robot.imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotLog.ii("CAL", "resetAngle - lastAngles = %2.2f", lastAngles.firstAngle);
        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double getAbsoluteAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }



    public void rotate(int degrees, double power)
    {
        RobotLog.ii("CAL", "Enter - rotate - degrees=%d, power=%f",
                degrees, power);

        // restart imu movement tracking.
        resetAngle();

        if (degrees < 0)
        {   // turn right.
            robot.turnRight(power);
        }
        else if (degrees > 0)
        {   // turn left.
            robot.turnLeft(power);
        }
        else return;


        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && !isStopRequested() && getAngle() == 0)
            {
            }

            while (opModeIsActive() && !isStopRequested() && getAngle() > degrees)
            {
            }
        }
        else    // left turn.
            while (opModeIsActive() && !isStopRequested() && getAngle() < degrees)
            {
            }

        // turn the motors off.
        power = 0;
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
        robot.backleftMotor.setPower(power);
        robot.backrightMotor.setPower(power);

        // wait for rotation to stop.
        sleep(50);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii("CAL", "Exit - rotate");
    }


      public void myEncoderDrive(Direction direction, double speed, double Inches, double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        //robot.leftMotor.setPower(speed);
        //robot.rightMotor.setPower(speed);
        //robot.backleftMotor.setPower(speed);
        //robot.backrightMotor.setPower(speed);
        //Reset the encoder
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.FORWARD) {
                //Go forward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.BACKWARD) {
                //Go backward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.STRAFE_RIGHT) {
                //Strafe Right
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.STRAFE_LEFT) {
                //Strafe Left
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

            } else {
                Inches = 0;
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }


            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);
            robot.backleftMotor.setTargetPosition(newLeftBackTarget);
            robot.backrightMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            robot.backleftMotor.setPower(Math.abs(speed));
            robot.backrightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy())) {
/*
                if (sensors_2_use == SensorsToUse.USE_COLOR)
                {

                }
                if (sensors_2_use == SensorsToUse.USE_DISTANCE)
                {
                    if (robot.sensorRange.getDistance(DistanceUnit.INCH) < 5) {
                        int count = 0;
                        while (opModeIsActive() && !isStopRequested() &&
                                (runtime.seconds() < timeoutS) &&
                                (count < 20) &&
                                (robot.sensorRange.getDistance(DistanceUnit.INCH) < 5)) {
                            sleep(100);
                            count++;
                        }
                    }

                }

                if (sensors_2_use == SensorsToUse.USE_TOUCH)
                {
                    robot.digitalTouch.setMode(DigitalChannel.Mode.INPUT);

                    if (robot.digitalTouch.getState() == true) {
                        telemetry.addData("Digital Touch", "Is Not Pressed");
                    } else {
                        telemetry.addData("Digital Touch", "Is Pressed");
                        break;
                    }
                    telemetry.update();

                }
                */

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            telemetry.addData("left", robot.leftMotor.getCurrentPosition());
            telemetry.addData("right", robot.rightMotor.getCurrentPosition());
            telemetry.addData("backl", robot.backleftMotor.getCurrentPosition());
            telemetry.addData("backr", robot.backrightMotor.getCurrentPosition());
            telemetry.update();

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);   // optional pause after each move
        }

        RobotLog.ii("CAL", "Exit - myEncoderDrive ");
    }
  /*  public void myEncoderLift(Direction direction, double speed, double Inches, double timeoutS)
    {
        int newLiftTarget = 0;
        int newLift2Target = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        robot.liftMotor.setPower(speed);
        robot.liftMotor2.setPower(speed);
        robot.liftMotor3.setPower(speed);

        //Reset the encoder
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.ROBOT_UP) {
                //Go up
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);

            } else if (direction == Direction.ROBOT_DOWN) {
                //Go down
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH_LIFT);

            } else {
                Inches = 0;
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
            }


            robot.liftMotor.setTargetPosition(newLiftTarget);
            robot.liftMotor2.setTargetPosition(newLift2Target);
            robot.liftMotor3.setTargetPosition(newLiftTarget);



            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();

            robot.liftMotor.setPower(Math.abs(speed));
            robot.liftMotor2.setPower(Math.abs(speed));
            robot.liftMotor3.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLiftTarget, newLift2Target);
                telemetry.addData("Path2", "Running at %7d :%7d", robot.liftMotor.getCurrentPosition(),
                        robot.liftMotor2.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.liftMotor.setPower(0);
            robot.liftMotor2.setPower(0);
            robot.liftMotor3.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(50);   // optional pause after each move
        }
    }
    public void myEncoderLift1(String d, double speed, double Inches, double timeoutS)
    {
        Direction direction = null;
        if (d == "ROBOT_UP"){
            direction = Direction.ROBOT_UP;
        } else if (d == "ROBOT_DOWN"){
            direction = Direction.ROBOT_DOWN;
        }
        int newLiftTarget = 0;
        int newLift2Target = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        robot.liftMotor.setPower(speed);
        robot.liftMotor2.setPower(speed);
        //Reset the encoder
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.ROBOT_UP) {
                //Go up
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);

            } else if (direction == Direction.ROBOT_DOWN) {
                //Go down
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH_LIFT);

            } else {
                Inches = 0;
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH_LIFT);
            }


            robot.liftMotor.setTargetPosition(newLiftTarget);
            robot.liftMotor2.setTargetPosition(newLift2Target);


            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            robot.liftMotor.setPower(Math.abs(speed));
            robot.liftMotor2.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLiftTarget, newLift2Target);
                telemetry.addData("Path2", "Running at %7d :%7d", robot.liftMotor.getCurrentPosition(),
                        robot.liftMotor2.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.liftMotor.setPower(0);
            robot.liftMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(50);   // optional pause after each move
        }
    }
    public void MasterEncoder(Direction direction, Direction LiftDirection, double speed, double LiftSpeed, double Inches, double LiftInches, double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int newLiftTarget = 0;
        int newLift2Target = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        robot.backleftMotor.setPower(speed);
        robot.backrightMotor.setPower(speed);
        robot.liftMotor.setPower(LiftSpeed);
        robot.liftMotor2.setPower(LiftSpeed);
        robot.liftMotor3.setPower(LiftSpeed);

        //Reset the encoder
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            if (LiftDirection == Direction.ROBOT_UP) {
                //Go up
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (-1 * LiftInches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (LiftInches * COUNTS_PER_INCH_LIFT);

            } else if (LiftDirection == Direction.ROBOT_DOWN) {
                //Go down
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (LiftInches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (-1 * LiftInches * COUNTS_PER_INCH_LIFT);

            } else {
                Inches = 0;
                newLiftTarget = robot.liftMotor2.getCurrentPosition() + (int) (LiftInches * COUNTS_PER_INCH_LIFT);
                newLift2Target = robot.liftMotor.getCurrentPosition() + (int) (LiftInches * COUNTS_PER_INCH_LIFT);
            }
            // Determine new target position, and pass to motor controller
            if (direction == Direction.FORWARD){
                //Go forward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.BACKWARD) {
                //Go backward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.STRAFE_RIGHT) {
                //Strafe Right
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            } else if (direction == Direction.STRAFE_LEFT) {
                //Strafe Left
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

            } else {
                Inches = 0;
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }



            robot.liftMotor.setTargetPosition(newLiftTarget);
            robot.liftMotor2.setTargetPosition(newLift2Target);
            robot.liftMotor3.setTargetPosition(newLiftTarget);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);
            robot.backleftMotor.setTargetPosition(newLeftBackTarget);
            robot.backrightMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.liftMotor.setPower(Math.abs(LiftSpeed));
            robot.liftMotor2.setPower(Math.abs(LiftSpeed));
            robot.liftMotor3.setPower(Math.abs(LiftSpeed));
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            robot.backleftMotor.setPower(Math.abs(speed));
            robot.backrightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftMotor.isBusy())||(robot.liftMotor.isBusy()))) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.liftMotor.setPower(0);
            robot.liftMotor2.setPower(0);
            robot.liftMotor3.setPower(0);

            telemetry.addData("left", robot.leftMotor.getCurrentPosition());
            telemetry.addData("right", robot.rightMotor.getCurrentPosition());
            telemetry.addData("backl", robot.backleftMotor.getCurrentPosition());
            telemetry.addData("backr", robot.backrightMotor.getCurrentPosition());
            telemetry.update();

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);   // optional pause after each move
        }

        //RobotLog.ii("CAL", "Exit - myEncoderDrive ");
    }
*/
}
