package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test tape measuring", group = "teleop")
public class testTapeMeasuring extends LinearOpMode {

    //imported hardware from "hardwareFF" public class:
    hardwareFF robot = new hardwareFF();
    double triggerDeadzone = 0.05;

    public void runOpMode() {
        //initialization code goes here
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            /**
             * Arm function
             */
            if (robot.bottomLimit.isPressed() && gamepad1.left_stick_y > 0) {
                robot.mtrArm.setPower(0);
            } else if (robot.bottomLimit.isPressed() && gamepad1.left_stick_y < 0) {
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            } else {
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            }
            /**
             * Turret function
             */
            if (robot.frontLimit.isPressed() && gamepad1.right_stick_x > 0) {
                robot.mtrTurret.setPower(0);
                telemetry.addLine("front limit stopping power");
            } else if (robot.backLimit.isPressed() && gamepad1.right_stick_x <0){
                robot.mtrTurret.setPower(0);
                telemetry.addLine("BACK limit stopping power");
            }
            else {
                robot.mtrTurret.setPower(gamepad1.right_stick_x);
            }
            /**
             * tilt controls
             */
            if (gamepad1.right_bumper) {
                robot.svoIntakeTilt.setPosition(variable.intakeInit);
            }
            if (gamepad1.left_bumper) {
                robot.svoIntakeTilt.setPosition(variable.intakeHigh);
            }
            /**
             * MEASURE NOW
             */
            robot.mtrTape.setPower(gamepad1.right_trigger);
            robot.mtrTape.setPower(-gamepad1.left_trigger);
            //if that didnt work, change to this:
            /*
            if(gamepad1.right_trigger > triggerDeadzone){
                robot.mtrTape.setPower(gamepad1.right_trigger);
            }else if(gamepad1.left_trigger > triggerDeadzone){
                robot.mtrTape.setPower(-gamepad1.left_trigger);
            }
                else{
                robot.mtrTape.setPower(0);
            }
             */

            telemetry.addData("x value of turret: ", gamepad1.right_stick_x);
            telemetry.addData("right trigger reading: ", gamepad1.right_trigger);
            telemetry.addData("left trigger reading: ", gamepad1.left_trigger);
            telemetry.addData("bottom limit status", robot.bottomLimit.isPressed());
            telemetry.addData("front limit status", robot.frontLimit.isPressed());
            telemetry.addData("back limit status", robot.backLimit.isPressed());
            telemetry.addData("mid limit status", robot.midLimit.isPressed());
            telemetry.addData("intake limit status", robot.intakeLimit.isPressed());
            telemetry.update();
        }
    }
}
