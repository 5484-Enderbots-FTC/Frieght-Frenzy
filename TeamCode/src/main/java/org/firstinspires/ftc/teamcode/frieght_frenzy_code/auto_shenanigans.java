/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.sql.DriverManager.println;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "auto_shenanigans", group = "Linear Opmode")

public class auto_shenanigans extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    hardwareFF craig = new hardwareFF();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        craig.init(hardwareMap);
        println("nyehehehehehhe im leaving some secrets here for future pogrammer -Tom, -Djordje, -Madeline");
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        craig.mtrFL.setDirection(DcMotor.Direction.FORWARD);
        craig.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        craig.mtrFR.setDirection(DcMotor.Direction.FORWARD);
        craig.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        craig.mtrBL.setDirection(DcMotor.Direction.FORWARD);
        craig.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        craig.mtrBR.setDirection(DcMotor.Direction.FORWARD);
        craig.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forward(20,2000);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private void forward(float power, int distance){
        int adjustmentA1=1;
        int adjustmentA2=0;

        double adjustmentB1=1;
        double adjustmentB2=0;

        craig.mtrFL.setPower((-power*adjustmentB1)+adjustmentB2);
        craig.mtrFL.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        craig.mtrFR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFR.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        craig.mtrBL.setPower((-power*adjustmentB1)+adjustmentB2);
        craig.mtrBL.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        craig.mtrBR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBR.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        while(craig.mtrFL.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrFR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBL.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craig.mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void backward(float power, int distance){
        int adjustmentA1=1;
        int adjustmentA2=0;

        double adjustmentB1=1;
        double adjustmentB2=0;

        craig.mtrFL.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFL.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);
        craig.mtrFR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFR.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);
        craig.mtrBL.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBL.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);
        craig.mtrBR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBR.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);

        craig.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrFR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBL.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craig.mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void left(float power, int distance){
        int adjustmentA1=1;
        int adjustmentA2=0;

        double adjustmentB1=1;
        double adjustmentB2=0;

        craig.mtrFL.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFL.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);
        craig.mtrFR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFR.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        craig.mtrBL.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBL.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        craig.mtrBR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBR.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);

        craig.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrFR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBL.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craig.mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void right(float power, int distance){
        int adjustmentA1=1;
        int adjustmentA2=0;

        double adjustmentB1=1;
        double adjustmentB2=0;

        craig.mtrFL.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFL.setTargetPosition((distance*adjustmentA1)+adjustmentA2);
        craig.mtrFR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrFR.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);
        craig.mtrBL.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBL.setTargetPosition((-distance*adjustmentA1)+adjustmentA2);
        craig.mtrBR.setPower((power*adjustmentB1)+adjustmentB2);
        craig.mtrBR.setTargetPosition((distance*adjustmentA1)+adjustmentA2);

        craig.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrFR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBL.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(craig.mtrBR.isBusy()) {
            //Loop body can be empty
        }
        craig.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craig.mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craig.mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
