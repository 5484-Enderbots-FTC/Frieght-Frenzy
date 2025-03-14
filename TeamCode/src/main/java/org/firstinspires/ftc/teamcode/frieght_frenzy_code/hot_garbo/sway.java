package org.firstinspires.ftc.teamcode.frieght_frenzy_code.hot_garbo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;

@TeleOp(name = "teleop ff2", group = "teleop")
public class sway extends LinearOpMode {
    //tell the code what motors and sensors you want to use (or just import the hardware teehee)
    hardwareFF robot = new hardwareFF();

    Boolean down = false;
    Boolean sway = true;
    double clock = 0;
    double swaynum = .15;


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
            while (sway && !isStopRequested()) {
                if (down) {
                    clock = clock - .0005;
                }
                if (!down) {
                    clock = clock + .0005;
                }
                if (clock > 8) {
                    down = true;
                }
                if (clock < 0) {
                    down = false;
                }
                if (clock < 8 && clock > 6) {
                    robot.mtrTurret.setPower(-2 * swaynum);
                }
                if (clock < 6 && clock > 4) {
                    robot.mtrTurret.setPower(-1 * swaynum);
                }
                if (clock < 4 && clock > 2) {
                    robot.mtrTurret.setPower(1 * swaynum);
                }
                if (clock < 2 && clock > 0) {
                    robot.mtrTurret.setPower(2 * swaynum);
                }
                ;
            }
            robot.mtrTurret.setPower(0);


            //we usually add some telemetry at the end to tell us useful information during testing :)
            telemetry.addData("left stick y value: ", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

}
