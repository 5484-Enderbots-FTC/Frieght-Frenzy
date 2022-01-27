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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;

@Autonomous(name = "autoRedCarousel no webcam")
public class AutoRedCarouselNoWeb extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    ElapsedTime duckTimer = new ElapsedTime();
    ElapsedTime dispenseTimer = new ElapsedTime();

    double duckTime = 5;
    double dispenseTime = 1.5;

    double runningOpMode = 3;

    Pose2d endDepPos = new Pose2d(0,0,0);
    double reverseHeading = Math.toRadians(0);
    turretState currentTurretState = turretState.NOTHING;
    armState currentArmState = armState.NOTHING;

    private enum turretState {
        NOTHING,
        POWER,
        FINDMID
    }

    private enum armState {
        NOTHING,
        RAISE,
        DISPENSE,
        WAIT
    }

    @Override
    public void runOpMode() {
        //this will init EVERYTHING on the robot
        robot.init(hardwareMap);
        //FFMecanum must be called AFTER the robot init bc just the motors need to be overridden.
        FFMecanumDrive drive = new FFMecanumDrive(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                /*
                .addTemporalMarker(0.9, 0, () -> {
                    robot.svoCarousel.setPower(1);
                    duckTimer.reset();
                })

                 */
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47),reverseHeading)
                .addTemporalMarker(0.1, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeHigh);
                })
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -52),reverseHeading)
                .addTemporalMarker(0.1, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeMid);
                })

                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47),reverseHeading)
                .addTemporalMarker(0.1, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeLow);
                })
                .build();

        Trajectory toPark1 = drive.trajectoryBuilder(toRedCarousel.end())
                .lineTo(traj.toParkPos1)
                .addTemporalMarker(0.5, () -> {
                    if (runningOpMode == 1) {
                        robot.svoIntakeTilt.setPosition(0.9);
                    }
                })
                .build();
        Trajectory toPark2 = drive.trajectoryBuilder(toPark1.end())
                .lineTo(traj.toParkPos2)
                .build();


        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            while(!robot.midLimit.isPressed()){
                robot.mtrTurret.setPower(-0.3);
            }
            robot.mtrTurret.setPower(0);

            drive.followTrajectory(toRedCarousel);
            robot.svoCarousel.setPower(1);
            sleep(2500);
            robot.svoCarousel.setPower(0);
            //duckTimer.reset();
            /*
            currentTurretState = turretState.POWER;
            switch (currentTurretState) {
                case NOTHING:
                    break;
                case POWER:
                    robot.mtrTurret.setPower(-0.3);
                    currentTurretState = turretState.FINDMID;
                    break;
                case FINDMID:
                    if (duckTimer.seconds() < duckTime) {
                        if (robot.midLimit.isPressed()) {
                            robot.mtrTurret.setPower(0);
                            telemetry.addLine("less than duck time mid pressed");
                            telemetry.update();
                        }
                    } else if(duckTimer.seconds() > duckTime){
                        robot.svoCarousel.setPower(0);
                        telemetry.addLine("carousel stopped");
                        telemetry.update();
                        if(robot.midLimit.isPressed()){
                            robot.mtrTurret.setPower(0);
                            telemetry.addLine("greater than duck time mid pressed");
                            telemetry.update();
                            currentTurretState = turretState.NOTHING;
                        }
                    }
                    break;
            }

             */

            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(runningOpMode == 3){
                robot.movearm(0.7, var.thirdLvl);
            }else if(runningOpMode == 2){
                robot.movearm(0.7, var.secondLvl);
            }else if (runningOpMode == 1){
                robot.movearm(0.7, var.firstLvl);
            }
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.mtrArm.isBusy()){

            }
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//            currentArmState = armState.RAISE;
/*
            switch (currentArmState) {
                case NOTHING:
                    break;
                case RAISE:
                    if (robot.mtrArm.isBusy()) {

                    } else {
                        robot.mtrArm.setPower(0);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentArmState = armState.DISPENSE;
                    }
                    break;
                case DISPENSE:
                    if (!drive.isBusy()) {
                        robot.svoIntake.setPower(-var.lessPower);
                        dispenseTimer.reset();
                        currentArmState = armState.WAIT;
                    } else {
                    }
                    break;
                case WAIT:
                    if (dispenseTimer.seconds() > dispenseTime) {
                        robot.svoIntake.setPower(0);
                        currentArmState = armState.NOTHING;
                    } else {
                    }
                    break;
            }

 */
            /*
            if (runningOpMode == 3) {
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                drive.followTrajectory(toRedHub3);
                reverseHeading = Math.toRadians(180);
                endDepPos = toRedHub3.end();
            } else if (runningOpMode == 2) {
                robot.svoIntakeTilt.setPosition(var.intakeMid);
                drive.followTrajectory(toRedHub2);
                endDepPos = toRedHub2.end();
            } else if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(var.intakeLow);
                drive.followTrajectory(toRedHub1);
                endDepPos = toRedHub1.end();
            }
            robot.svoIntake.setPower(-var.lessPower);
            sleep(1500);
            robot.svoIntake.setPower(0);
            if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(0.9);
            }

             */
            //TODO: add the move arm encoder positions to the runningOpMode ifs above
            drive.followTrajectory(toPark1);
            drive.followTrajectory(toPark2);
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
            break;
        }
    }
}