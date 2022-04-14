package org.firstinspires.ftc.teamcode.frieght_frenzy_code.hot_garbo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

@Disabled
@TeleOp(name = "why servo no work", group = "teleop")
public class tESt_mOmEnT extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    private static double reset = 0.28;
    private static double low = 0;
    private static double inc = 0.005;
    private double tiltNumber = 0.28;

    boolean freightCollected = false;
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.svoIntakeTilt.setPosition(var.intakeCollectTeleop);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(gamepad2.a){
                robot.svoIntake.setPower(var.lessPower);
            }

            if(gamepad2.b){
                robot.svoIntake.setPower(-var.lessPower);
            }

            /**
             * Turret Movement ~
             */

            if (robot.frontLimit.isPressed() && gamepad1.right_stick_x > 0) {
                robot.mtrTurret.setPower(0);
                telemetry.addLine("front limit stopping power");
            } else if (robot.backLimit.isPressed() && gamepad1.right_stick_x < 0) {
                robot.mtrTurret.setPower(0);
                telemetry.addLine("BACK limit stopping power");
            } else {
                robot.mtrTurret.setPower(gamepad1.right_stick_x);
            }

            robot.mtrTape.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            telemetry.addData("is the servo supposed to be moving", robot.svoIntake.getPower());
            telemetry.addData("tape encoder reading: ", robot.mtrTape.getCurrentPosition());
            telemetry.addData("arm encoder reading: ", robot.mtrArm.getCurrentPosition());

            telemetry.update();
        }

    }
}
