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

package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;

@Disabled
@Autonomous(name = "intaque")
public class ForwardUntilIntake extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode;
    Pose2d endDepPos;
    Pose2d intakeEnd;
    ElapsedTime intakeDistance = new ElapsedTime();
    double distanceTravelled = 0;
    double maxVel = 10;
    double acc = 0;


    @Override
    public void runOpMode() {
        //this will init EVERYTHING on the robot
        robot.init(hardwareMap);
        robot.initWebcam();
        //FFMecanum must be called AFTER the robot init bc just the motors need to be overridden.
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory intakeForward = drive.trajectoryBuilder(traj.startPoseRC)
                .forward(20,  FFMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {robot.svoIntake.setPower(variable.intakeCollect);})
                .build();



        waitForStart();
        while (opModeIsActive()) {
            drive.followTrajectoryAsync(intakeForward);
            intakeDistance.reset();
            while (robot.intakeLimit.isPressed()){
                telemetry.addLine("consuming");
                if (!robot.intakeLimit.isPressed()){
                    telemetry.addLine("consumed");
                    robot.svoIntake.setPower(0);
                    acc = DriveConstants.MAX_ACCEL;
                    distanceTravelled = maxVel*intakeDistance.time();// + 0.5*acc*(Math.pow(intakeDistance.time(),2));
                    intakeEnd = new Pose2d(40+distanceTravelled, -67,Math.toRadians(180));
                    drive.cancelFollowing();
                    break;
                }
                drive.update();
                telemetry.update();
            }

            break;
        }
    }
    public void spitOutBlock (){
        if (runningOpMode ==3 ){
            robot.svoIntakeTilt.setPosition(variable.intakeHigh);
        }
        else if (runningOpMode == 2){
            robot.svoIntakeTilt.setPosition(variable.intakeMid);
        }
        else if (runningOpMode  == 1){
            robot.svoIntakeTilt.setPosition(variable.intakeLow);
        }
        sleep(1000);
        robot.svoIntake.setPower(-variable.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}