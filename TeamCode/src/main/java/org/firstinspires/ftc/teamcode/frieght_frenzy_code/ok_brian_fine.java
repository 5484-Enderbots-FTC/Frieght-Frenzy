package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ok brian", group = "teleop")
public class ok_brian_fine extends LinearOpMode {
    //tell the code what motors and sensors you want to use (or just import the hardware teehee)
    hardwareFF robot = new hardwareFF();

    public void runOpMode() {
        //init code goes here
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //all teleop code after start button pressed goes here

            //FINE BRIAN HERES THE CONTROLS
            robot.updateDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            telemetry.addLine("hey brian here's your freaking controls .__.");
            telemetry.update();
        }
    }

}
