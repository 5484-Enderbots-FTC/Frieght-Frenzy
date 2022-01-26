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

    }

}
