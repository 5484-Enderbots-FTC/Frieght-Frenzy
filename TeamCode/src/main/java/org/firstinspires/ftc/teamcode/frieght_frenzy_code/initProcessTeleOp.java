package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "initProcess", group = "teleop")
public class initProcessTeleOp extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    private static double reset = 1;
    private static double inc = 0.005;
    private double tiltNumber = 0;
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.svoIntakeTilt.setPosition(reset);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            /**
             * Arm Movement ~
             */
            if(robot.bottomLimit.isPressed() && gamepad1.left_stick_y > 0){
                robot.mtrArm.setPower(0);
            }
            else if(robot.bottomLimit.isPressed() && gamepad1.left_stick_y < 0){
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            }
            else{
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            }

            /**
             * Turret Movement ~
             */
            robot.mtrTurret.setPower(gamepad1.right_stick_x);

            /**
             * Intake Tilting Servo ~
             */
            if(gamepad1.a){
                tiltNumber = tiltNumber + inc;
                robot.svoIntakeTilt.setPosition(tiltNumber);
            }
            if(gamepad1.b){
                tiltNumber = reset;
                robot.svoIntakeTilt.setPosition(reset);
            }

            telemetry.addData("MidLimit? ", robot.midLimit.isPressed());
            telemetry.addData("MidLimit? ", robot.rightLimit.isPressed());
            //telemetry.addData("MidLimit? ", robot.leftLimit.isPressed());
            telemetry.addData("Servo current pos: ", robot.svoIntakeTilt.getPosition());
            telemetry.addData("Servo input number: ", tiltNumber);
            telemetry.update();
        }

    }
}
