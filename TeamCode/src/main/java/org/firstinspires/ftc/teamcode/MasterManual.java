package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@TeleOp(name = "Master Manual", group = "Linear Opmode")
//@Disabledpublic

public class MasterManual extends LinearOpMode {

    MasterHW robot = new MasterHW();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        double motor_power = 0.3;

        double slide_power = 0.8;

        float leftX, leftY, rightZ;

        final float[] hsvValues = new float[3];


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        //waitForStart();
        // Do not use waitForStart() if you have Motorola E4 phones.
        waitForStart();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if ((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.right_stick_x != 0)) {
                leftY = gamepad1.left_stick_y;
                leftX = gamepad1.left_stick_x * -1;
                rightZ = gamepad1.right_stick_x * -1;
                robot.moveHolonomic(leftX, leftY, rightZ);
            } else if (gamepad1.dpad_down) {
                //forward
                robot.moveHolonomic(0, motor_power * 1, 0);
            } else if (gamepad1.dpad_up) {
                //backwards
                robot.moveHolonomic(0, motor_power * -1, 0);
            } else if (gamepad1.dpad_left) {
                //rotate counter-clockwise
                robot.moveHolonomic(0, 0, motor_power * 1);
            } else if (gamepad1.dpad_right) {
                //rotate clockwise
                robot.moveHolonomic(0, 0, motor_power * -1);
            } else  if (gamepad2.dpad_down) {
                //slide up
                robot.liftMotor.setPower(slide_power);
                robot.liftMotor2.setPower(slide_power*-1);
            }
            else  if (gamepad2.dpad_up) {
                //slide down
                robot.liftMotor.setPower(slide_power*-1);
                robot.liftMotor2.setPower(slide_power);
            }
            else  if (gamepad2.x) {
                //claw open
                robot.lClaw.setPosition(0);
                robot.rClaw.setPosition(0);
            } else  if (gamepad2.b) {
                //claw close
                robot.lClaw.setPosition(0.7);
                robot.rClaw.setPosition(0.6);
            }
            else{
                robot.stopAllMotors();
                robot.stopLiftMotors();
                // robot.colorR1.enableLed(true);
                telemetry.update();
            }
        }
    }

}
