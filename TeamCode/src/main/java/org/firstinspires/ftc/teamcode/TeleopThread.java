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
                    robot.inTake.setPower(slide_power*-1);

                }
                else  if (gamepad2.dpad_right) {
                    robot.inTake.setPower(slide_power);

                }
                else  if (gamepad2.dpad_up) {
                    robot.liftMotor1.setPower(slide_power);

                }
                else  if (gamepad2.dpad_down) {
                    robot.liftMotor1.setPower(slide_power*-1);

                }
                else  if (gamepad2.x) {
                    //claw open
                    /*robot.lClaw.setPosition(0);
                    robot.rClaw.setPosition(0);*/
                    //robot.xClaw.setPosition(0.1);
                }
                else  if (gamepad2.b) {
                    //claw close
                    /*robot.lClaw.setPosition(0.7);
                 //   robot.rClaw.setPosition(0.7);*/
                    //robot.xClaw.setPosition(0.3);
                }
             /*   else  if (gamepad2.y) {
                    //claw close
                    /*robot.lClaw.setPosition(0.7);
                    robot.rClaw.setPosition(0.7);
                   robot.xSlide.setPosition(0);
                }
                else  if (gamepad2.a) {
                    //claw close
                    /*robot.lClaw.setPosition(0.7);
                    robot.rClaw.setPosition(0.7);
                    robot.xSlide.setPosition(0.35);
                }*/
                else  if (gamepad2.left_bumper) {
                    //claw close
                    /*robot.lClaw.setPosition(0.7);
                    robot.rClaw.setPosition(0.7);*/
                    runtime.reset();
                    while (runtime.seconds() <2)
                    {
                       // robot.liftMotor.setPower(slide_power*-1);
                       // robot.liftMotor2.setPower(slide_power);
                       // robot.liftMotor3.setPower(slide_power*-1);
                        }
                    }
                else  if (gamepad2.right_bumper) {
                    runtime.reset();
                    while (runtime.seconds() <1)
                    {
                        //robot.liftMotor.setPower(slide_power*-1);
                      //  robot.liftMotor2.setPower(slide_power);
                       // robot.liftMotor3.setPower(slide_power*-1);
                        }
                }

                else{
                     robot.stopAllMotors();
                   robot.stopLiftMotors();

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
                        leftY = gamepad1.left_stick_y;
                        leftX = gamepad1.left_stick_x * -1;
                        rightZ = gamepad1.right_stick_x * -1;
                        robot.moveHolonomic(leftX, leftY, rightZ);
                    } else

                    if (gamepad1.dpad_down) {
                        //forward
                        robot.moveForward(motor_power);
                       // robot.moveHolonomic(0, motor_power * 1, 0);
                    } else if (gamepad1.dpad_up) {
                        //backwards
                        //robot.moveHolonomic(0, motor_power * -1, 0);
                        robot.moveBackwards(motor_power);
                    } else if (gamepad1.dpad_left) {
                        //rotate counter-clockwise
                        robot.moveHolonomic(0, 0, motor_power * 1);
                    } else if (gamepad1.dpad_right) {
                        //rotate clockwise
                        robot.moveHolonomic(0, 0, motor_power * -1);
                    }  else{
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
