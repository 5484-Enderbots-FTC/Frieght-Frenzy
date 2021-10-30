package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop ff", group = "teleop")
public class teleop_two_remotes extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    ElapsedTime toggleBabyTimer = new ElapsedTime();
    
    boolean babyMode = false;
    
    public void runOpMode() {
        //init code goes here
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //all teleop code after start button pressed goes here

            //make robot wheels go brrr
            if(gamepad1.left_bumper && !babyMode && toggleBabyTimer.seconds() > var.toggleWait){
                //activate baby slow mode when left bumper is pressed
                babyMode = true;
                toggleBabyTimer.reset();
            }
            if(gamepad1.left_bumper && babyMode && toggleBabyTimer.seconds() > var.toggleWait){
                //deactivate baby slow mode by pressing left bumper again
                babyMode = false;
                toggleBabyTimer.reset();
            }
            
            if (!babyMode) {
                robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if(babyMode){
                robot.updateDrive(gamepad1.left_stick_y*0.5, gamepad1.left_stick_x*0.5, gamepad1.right_stick_x*0.5);
            }
            
            


            //we usually add some telemetry at the end to tell us useful information during testing :)
            telemetry.addData("left stick y value: ", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

}
