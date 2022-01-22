package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "initProcessAuto", group = "auto")
public class initProcessAuto extends LinearOpMode {
    hardwareFF robot = new hardwareFF();

    boolean armRunning = false;

    initState currentState = initState.NOTHING;

    private enum initState {
        NOTHING,
        SET_TURRET,
        SET_ARM,
        WAIT,
        FINISH
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.mtrTurret.setPower(0.7);
            currentState = initState.SET_TURRET;
            switch (currentState) {
                case NOTHING:
                    break;
                case SET_TURRET:
                    if (robot.rightLimit.isPressed()) {
                        robot.mtrTurret.setPower(0);
                        robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        currentState = initState.SET_ARM;
                    }
                    break;
                case SET_ARM:
                    if(!armRunning){
                        robot.mtrArm.setPower(-0.4);
                        armRunning = true;
                    }
                    if(robot.bottomLimit.isPressed()){
                        robot.mtrArm.setPower(0);
                        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.movearm(0.7,var.secondLvl);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentState = initState.WAIT;
                    }
                    break;
                case WAIT:
                    if(robot.mtrArm.isBusy()){

                    }else{
                        currentState = initState.FINISH;
                    }
                    break;
                case FINISH:
                    robot.mtrArm.setPower(0);
                    robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = initState.NOTHING;
                    break;

            }
            telemetry.update();
            break;
        }

    }
}
