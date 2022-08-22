/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.frieght_frenzy_code.hot_garbo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.vari;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple stones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@Autonomous
@Disabled
public class DistanceSensorAlign extends LinearOpMode
{
    hardwareFF robot = new hardwareFF();
    int turnDirection = 1;
    double leftDistance = 0;
    double rightDistance = 0;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */
        int alliance_element_location = 0;

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        robot.svoIntakeTilt.setPosition(vari.intakeInit);

        telemetry.update();
        telemetry.addData("Left Distance: ", robot.leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance: ", robot.rightDistance.getDistance(DistanceUnit.CM));
        leftDistance = robot.leftDistance.getDistance(DistanceUnit.CM);
        rightDistance = robot.rightDistance.getDistance(DistanceUnit.CM);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            if (robot.leftDistance.getDistance(DistanceUnit.CM) > 50){

            }
            else{

                leftDistance = robot.leftDistance.getDistance(DistanceUnit.CM);
            }
            if (robot.rightDistance.getDistance(DistanceUnit.CM) > 50){

            }
            else{
                rightDistance = robot.rightDistance.getDistance(DistanceUnit.CM);
            }
            if (robot.leftDistance.getDistance(DistanceUnit.CM)> robot.rightDistance.getDistance(DistanceUnit.CM)){
                turnDirection = -1;
            }
            else if (robot.leftDistance.getDistance(DistanceUnit.CM) < robot.rightDistance.getDistance(DistanceUnit.CM)){
                turnDirection = 1;
            }
            while (Math.abs(robot.leftDistance.getDistance(DistanceUnit.CM) - robot.rightDistance.getDistance(DistanceUnit.CM))>2){

                robot.powerTurn(0.5*turnDirection);
                telemetry.addData("Left Distance: ", robot.leftDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Right Distance: ", robot.rightDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            robot.brake();

        }
    }
}