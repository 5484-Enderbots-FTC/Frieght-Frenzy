package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "initProcess", group = "teleop")
public class initProcessTeleOp extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    private static double reset = 0.35;
    private static double low = 0;
    private static double inc = 0.005;
    private double tiltNumber = 0;

    boolean freightCollected = false;
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.svoIntakeTilt.setPosition(reset);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(gamepad1.dpad_up){
                robot.svoIntake.setPower(-vari.lessPower);
            }
            if (!robot.intakeLimit.isPressed()) {
                freightCollected = true;
            } else {
                freightCollected = false;
            }
            if(freightCollected){
                robot.svoIntake.setPower(vari.stop);
            }
            if(gamepad1.y){
                robot.svoIntake.setPower(vari.lessPower);
            }
            if(gamepad1.x){
                robot.svoIntake.setPower(vari.stop);
            }
            if(robot.bottomLimit.isPressed() && gamepad1.left_stick_y > 0){
                robot.mtrArm.setPower(0);
            }
            else if(robot.bottomLimit.isPressed() && gamepad1.left_stick_y < 0){
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            }
            else{
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            }
            if(robot.bottomLimit.isPressed()){
                robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            robot.updateDrive(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);

//ahahahahahhah
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
            if(gamepad1.right_bumper){
                robot.svoIntakeTilt.setPosition(vari.intakeCollect);
            }

            if(gamepad2.a){
                robot.mtrIntake.setPower(0.2);
            }

            robot.mtrTape.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            telemetry.addData("MidLimit? ", robot.midLimit.isPressed());
            telemetry.addData("Back limit? ", robot.backLimit.isPressed());
            telemetry.addData("Front limit? ", robot.frontLimit.isPressed());

            telemetry.addData("Servo current pos: ", robot.svoIntakeTilt.getPosition());
            telemetry.addData("Arm Position: ", robot.mtrArm.getCurrentPosition());

            telemetry.addData("Servo input number: ", tiltNumber);
            telemetry.update();
        }

    }
}
