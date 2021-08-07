package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "teleop ff", group = "teleop")
public class teleop_two_remotes extends LinearOpMode {
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

            //make robot wheels go brrr
            robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //we usually add some telemetry at the end to tell us useful information during testing :)
            telemetry.addData("left stick y value: ", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

}
