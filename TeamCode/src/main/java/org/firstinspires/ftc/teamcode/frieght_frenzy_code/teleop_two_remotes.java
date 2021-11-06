package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop ff", group = "teleop")
public class teleop_two_remotes extends LinearOpMode {

    //imported hardware from "hardwareFF" public class:
    hardwareFF robot = new hardwareFF();

    //this is the timer used to create a toggle switch:
    ElapsedTime toggleBabyTimer = new ElapsedTime();
    ElapsedTime toggleCarousel = new ElapsedTime();

    //this boolean keeps track of whether or not the toggle is on or off
    boolean babyMode = false;
    boolean carouselSpinning = false;
    
    public void runOpMode() {
        //initialization code goes here
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

            if(gamepad1.a){
                carouselSpinning = true;
                if(robot.alliance_switch.getState() == true){
                    robot.svoCarousel.setPower(1);
                }
                else{
                    robot.svoCarousel.setPower(-1);
                }

            }
            if(gamepad1.b){
                carouselSpinning = false;
                robot.svoCarousel.setPower(0);
            }


            //we usually add some telemetry at the end to tell us useful information during testing :)
            if(babyMode){
                telemetry.addLine("baby mode activated");
            }
            else{
                telemetry.addLine("baby mode inactive");
            }
            if(robot.alliance_switch.getState() == true) {
                telemetry.addLine("red alliance");
            }
            else {
                telemetry.addLine("blue alliance");
                telemetry.update();
            }
        }
    }

}
