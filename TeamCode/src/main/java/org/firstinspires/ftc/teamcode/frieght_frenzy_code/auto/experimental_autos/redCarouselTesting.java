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

package org.firstinspires.ftc.teamcode.frieght_frenzy_code.auto.experimental_autos;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.autoTrajectories;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.variable;

@Autonomous(name = "red carousel testing", group = "red")
public class redCarouselTesting extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode = 3;
    ElapsedTime duckTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(traj.redCarousel, Math.toRadians(180))
                .addDisplacementMarker(0.5,0, () -> {
                            robot.svoCarousel.setPower(1);
                            robot.mtrTurret.setPower(-0.4);
                        }
                )
                .build();

        robot.svoIntakeTilt.setPosition(variable.intakeInit);

        waitForStart();

        /**
         * this was so easy holy crap omg
         */

        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.movearm(variable.armInitPower, variable.thirdLvl);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(toRedCarousel);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        duckTime.reset();
        while (!robot.midLimit.isPressed() | duckTime.seconds() < 3) {
            if (robot.midLimit.isPressed()) {
                robot.mtrTurret.setPower(0);
            }
            if (duckTime.seconds() > 3) {
                robot.svoCarousel.setPower(0);
            }
        }



/*
while (!robot.midLimit.isPressed()) {
            robot.mtrTurret.setPower(-0.4);
        }
        robot.mtrTurret.setPower(0);


 */
        //robot.mtrArm.setPower(0);


        /**
         * shmove on to carousel and spain without the a
         */


        /*
        robot.svoCarousel.setPower(1);
        drive.setPoseEstimate(traj.redCarouselReset);
        drive.updatePoseEstimate();
        sleep(3000);
        robot.svoCarousel.setPower(0);
         */
    }
}
