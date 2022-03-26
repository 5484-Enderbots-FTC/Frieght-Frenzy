package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "test da angle", group = "teleop")
public class test_da_angle extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    private static double reset = 0;
    private static double low = 0;
    private static double inc = 0.005;
    boolean babyMode = false;
    ElapsedTime toggleBabyTimer = new ElapsedTime();

    private double tiltNumber = 0;
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if (gamepad1.left_bumper && !babyMode && toggleBabyTimer.seconds() > var.toggleWait) {
                //activate baby slow mode when left bumper is pressed
                babyMode = true;
                toggleBabyTimer.reset();
            }
            if (!babyMode) {
                robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if (babyMode) {
                robot.updateDrive(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
            }
            /**
             * Arm Movement ~
             */
            if(robot.bottomLimit.isPressed() && gamepad2.left_stick_y > 0){
                robot.mtrArm.setPower(0);
            }
            else if(robot.bottomLimit.isPressed() && gamepad2.left_stick_y < 0){
                robot.mtrArm.setPower(gamepad2.left_stick_y);
            }
            else{
                robot.mtrArm.setPower(gamepad2.left_stick_y);
            }

            /**
             * Turret Movement ~
             */
            robot.mtrTurret.setPower(gamepad2.right_stick_x);

            /**
             * Intake Tilting Servo ~
             */
            if(gamepad2.a){
                tiltNumber = tiltNumber + inc;
                robot.svoIntakeTilt.setPosition(tiltNumber);
            }
            if(gamepad2.b){
                tiltNumber = reset;
                robot.svoIntakeTilt.setPosition(reset);
            }
            if(gamepad2.right_bumper){
                robot.svoIntakeTilt.setPosition(var.intakeCollect);
            }
            robot.mtrTape.setPower(gamepad2.right_trigger);
            robot.mtrTape.setPower(-gamepad2.left_trigger);

            telemetry.addData("MidLimit? ", robot.midLimit.isPressed());
            telemetry.addData("Back limit? ", robot.backLimit.isPressed());
            telemetry.addData("Front limit? ", robot.frontLimit.isPressed());

            telemetry.addData("Servo current pos: ", robot.svoIntakeTilt.getPosition());
            telemetry.addData("Servo input number: ", tiltNumber);
            telemetry.update();
        }

    }
}
