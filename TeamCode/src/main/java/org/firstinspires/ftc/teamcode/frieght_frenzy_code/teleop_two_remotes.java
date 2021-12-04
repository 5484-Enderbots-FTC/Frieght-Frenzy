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


            /***
             * GAMEPAD 1 CONTROLS
             */

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
                    robot.svoCarousel.setPower(var.fullPower);
                }
                else{
                    robot.svoCarousel.setPower(-var.fullPower);
                }

            }
            if(gamepad1.b){
                carouselSpinning = false;
                robot.svoCarousel.setPower(var.stop);
            }

            /***
             * GAMEPAD 2 CONTROLS
             */

            //turret spin to da right
            if(gamepad2.left_stick_x > 0 && robot.rightLimit.isPressed()){
                robot.mtrTurret.setPower(-0.1);
            }else {
                robot.mtrTurret.setPower(gamepad2.left_stick_x);
            }
            //turret spin to da left
            if(gamepad2.left_stick_x < 0 && robot.leftLimit.isPressed()){
                robot.mtrTurret.setPower(0.1);
            }else{
               robot.mtrTurret.setPower(gamepad2.left_stick_x);
            }

 /*
            robot.mtrTurret.setPower(gamepad2.left_stick_x);
            robot.mtrArm.setPower(gamepad2.right_stick_y);

  */


            //arm go up
            if(gamepad2.right_stick_y > 0 && robot.topLimit.isPressed()){
                robot.mtrArm.setPower(var.stop);
            }else{
                robot.mtrArm.setPower(gamepad2.right_stick_y);
            }
            //arm go down
            if(gamepad2.right_stick_y < 0 && robot.bottomLimit.isPressed()){
                robot.mtrArm.setPower(var.stop);
            }else{
                robot.mtrArm.setPower(gamepad2.right_stick_y);
            }

            //servo tilt down
            if(gamepad2.left_bumper){
                robot.svoIntakeTilt.setPosition(var.intakeTiltDown);
            }

            //servo tilt up
            if(gamepad2.right_bumper){
                robot.svoIntakeTilt.setPosition(var.intakeTiltUp);
            }

            //run intake
            if(gamepad2.a){
                robot.svoIntake.setPower(var.fullPower);
            }
            //reverse intake
            if(gamepad2.b){
                robot.svoIntake.setPower(-var.fullPower);
            }
            if(gamepad2.x){
                robot.svoIntake.setPower(var.stop);
            }

/*
            //we usually add some telemetry at the end to tell us useful information during testing :)
            if(babyMode){
                telemetry.addLine("baby mode activated");
            }
            else{
                telemetry.addLine("baby mode inactive");
            }

 */
            if(robot.alliance_switch.getState() == true) {
                telemetry.addLine("red alliance");
            }
            else {
                telemetry.addLine("blue alliance");
            }
            if(robot.position_switch.getState() == true) {
                telemetry.addLine("carousel side");
            }
            else {
                telemetry.addLine("warehouse side");
            }

            telemetry.addData("top limit status", robot.topLimit.getValue());
            telemetry.addData("bottom limit status", robot.bottomLimit.getValue());
            telemetry.addData("right limit status", robot.rightLimit.isPressed());
            telemetry.addData("left limit status", robot.leftLimit.isPressed());


            telemetry.update();
        }



    }

}
