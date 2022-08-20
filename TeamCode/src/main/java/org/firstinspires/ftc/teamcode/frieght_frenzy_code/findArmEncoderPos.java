package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "find arm encoder", group = "teleop")
public class findArmEncoderPos extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    ElapsedTime buttonPress = new ElapsedTime();

    private static double buttonPressTime = 0.5;
    private static int reset = 0;
    private static double low = 0;
    private static double inc = 0.005;
    private double tiltNumber = 0;

    private int armEncoderCount = 0;
    private static int armInc = 150;

    boolean zeroPosSet = false;

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.svoIntakeTilt.setPosition(variable.intakeInit);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            /**
             * Arm Movement ~
             */
            if (robot.bottomLimit.isPressed() && !zeroPosSet) {
                robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                zeroPosSet = true;
            }
            if (robot.bottomLimit.isPressed() && gamepad1.left_stick_y > 0) {
                robot.mtrArm.setPower(0);
            } else if (robot.bottomLimit.isPressed() && gamepad1.left_stick_y < 0) {
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            } else {
                robot.mtrArm.setPower(gamepad1.left_stick_y);
            }

            /**
             * Arm encoder movement :3
             */
            if (gamepad1.right_bumper && buttonPress.seconds() > buttonPressTime) {
                buttonPress.reset();
                armEncoderCount += armInc;
            }
            if (gamepad1.left_bumper && buttonPress.seconds() > buttonPressTime) {
                buttonPress.reset();
                armEncoderCount -= armInc;
            }
            if (gamepad1.x) {
                armEncoderCount = reset;
            }
            if (gamepad1.y) {
                robot.movearm(0.7, armEncoderCount);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(robot.mtrArm.isBusy()){
                    telemetry.addLine("arm go brrr");
                    telemetry.addData("current arm encoder: ", robot.mtrArm.getCurrentPosition());
                }
                robot.mtrArm.setPower(0);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            /**
             * Turret Movement ~
             */
            robot.mtrTurret.setPower(gamepad1.right_stick_x);

            /**
             * Intake Tilting Servo ~
             */
            if (gamepad1.a) {
                tiltNumber += inc;
                robot.svoIntakeTilt.setPosition(tiltNumber);
            }
            if (gamepad1.b) {
                tiltNumber = reset;
                robot.svoIntakeTilt.setPosition(reset);
            }

            telemetry.addData("MidLimit? ", robot.midLimit.isPressed());
            telemetry.addData("Back limit? ", robot.backLimit.isPressed());
            telemetry.addData("Front limit? ", robot.frontLimit.isPressed());
            telemetry.addLine("");
            telemetry.addData("Servo current pos: ", robot.svoIntakeTilt.getPosition());
            telemetry.addData("Servo input number: ", tiltNumber);
            telemetry.addLine("");
            telemetry.addData("Arm current encoder number: ", armEncoderCount);
            telemetry.addData("actual arm encoder: ", robot.mtrArm.getCurrentPosition());
            telemetry.update();
        }

    }
}
