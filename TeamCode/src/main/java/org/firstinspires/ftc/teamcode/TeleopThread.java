package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "Teleop Manual", group = "Linear Opmode")
//@Disabledpublic

public class TeleopThread extends LinearOpMode {
    MasterHW robot = new MasterHW();
    private ElapsedTime runtime = new ElapsedTime();
    double motor_power = 0.3;

    double slide_power = 0.9;

    float leftX, leftY, rightZ;

    public TeleopThread() throws Exception
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "RunMode");
        telemetry.update();
        robot.init(hardwareMap);

        robot.Claw.setPosition(0);
        robot.rRotate.setPosition(0.65);
        robot.Drone2.setPosition(0);
        robot.servoArm.setPosition(0.16);

        Thread  driveThread = new DriveThread();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();
       // robot.xRotate.setPosition(0.4);

        // start the driving thread.

        driveThread.start();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        runtime.reset();
        try
        {
            while (opModeIsActive())
            {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();

                if (gamepad2.dpad_left) {
                    //robot.inTake.setPower(slide_power*-1);
                    robot.rRotate.setPosition(.1);
                }
                else  if (gamepad2.dpad_right) {
                  //  robot.inTake.setPower(slide_power);
robot.rRotate.setPosition(0.65);
                }
                else  if (gamepad2.dpad_up) {
                    robot.liftMotor1.setPower(slide_power);
                    robot.liftMotor2.setPower(slide_power*-1);

                }
                else  if (gamepad2.dpad_down) {
                    robot.liftMotor1.setPower(slide_power * -1);
                    robot.liftMotor2.setPower(slide_power);
                }
                else  if (gamepad2.x) {
                    robot.rRotate.setPosition(0);
                } else  if (gamepad2.b) {
                    robot.rRotate.setPosition(0.25);
                }
                else if (gamepad2.y){
                   robot.rClaw.setPosition(0.0);
                    robot.Claw.setPosition(.25);
                }
                else if (gamepad2.a){
                    robot.rClaw.setPosition(0.25);
                    robot.Claw.setPosition(0);
                }
                else if (gamepad1.a){
                    robot.servoArm.setPosition(0.16);
                }
                else if (gamepad1.y){
                    robot.servoArm.setPosition(0);
                }
                else  if (gamepad2.left_bumper) {
                    robot.mHook.setPower(slide_power);
                }
                else  if (gamepad2.right_bumper) {
                    robot.mHook.setPower(slide_power * -1);
                }
                else if (gamepad1.left_bumper) {
                    robot.Drone2.setPosition(0);
                }
                else if (gamepad1.right_bumper) {
                    robot.Drone2.setPosition(0.3);
                }
               else if (gamepad2.right_trigger>0.5) {
                    robot.rClaw.setPosition(0.10);
                }
               else if (gamepad2.left_trigger>0.5) {
                    robot.Claw.setPosition(.15);
                }
                else{
                  //  robot.stopAllMotors();
                    robot.stopLiftMotors();
                    // robot.colorR1.enableLed(true);
                    telemetry.update();
                }
            }
        }  catch(Exception e) {telemetry.addData("Status", "Run Time: " + e.getMessage());
            telemetry.update();}
        driveThread.interrupt();

    }

    private class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");

            // Logging.log("%s", this.getName());
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            telemetry.addData("Status", "Thread Start");
            telemetry.update();

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    if ((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.right_stick_x != 0)) {
                        leftY = gamepad1.left_stick_y *-1;
                        leftX = gamepad1.left_stick_x ;
                        rightZ = gamepad1.right_stick_x *-1;
                        robot.moveHolonomic(leftX, leftY, rightZ);
                    } else

                    if (gamepad1.dpad_down) {
                        //forward
                        robot.moveBackwards(motor_power);
                       // robot.moveHolonomic(0, motor_power * 1, 0);
                    } else if (gamepad1.dpad_up) {
                        //backwards
                        //robot.moveHolonomic(0, motor_power * -1, 0);
                        robot.moveForward(motor_power);
                    } else if (gamepad1.x) {
                        //rotate counter-clockwise
                        robot.moveHolonomic(0, 0, motor_power * 1);
                    } else if (gamepad1.b) {
                        //rotate clockwise
                        robot.moveHolonomic(0, 0, motor_power * -1);
                    }
                    else if (gamepad1.dpad_left) {
                        robot.strafeleft(motor_power);
                    } else if (gamepad1.dpad_right){
                        robot.strafeRight(motor_power);
                    }else{
                        robot.stopAllMotors();}

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //    catch (InterruptedException e) {
            //      telemetry.addData("Status", "interrupted" + this.getName());
            //    telemetry.update();}
            // an error occurred in the run loop.
            catch (Exception e) {
                telemetry.addData("Status", "error in Thread" + this.getName());
                telemetry.update();}

            telemetry.addData("Status", "End of thread" + this.getName());
            telemetry.update();
        }
    }
}
