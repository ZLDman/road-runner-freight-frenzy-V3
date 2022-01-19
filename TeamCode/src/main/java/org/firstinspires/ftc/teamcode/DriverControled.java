package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class DriverControled extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / 2,
                            -gamepad1.left_stick_x / 2,
                            -gamepad1.right_stick_x / 3
                    )
            );

            if (gamepad2.x) {
                robot.openClaw();
            }
            if (gamepad2.y) {
                robot.closeClaw();
            }

            if (gamepad2.dpad_down) {
                robot.setTarget(Robot.target.intake);
                //robot.setLiftPos(0);
            }
            if (gamepad2.dpad_up) {
                robot.setTarget(Robot.target.place);
                //robot.setLiftPos(800);
            }

            if (gamepad2.dpad_left) {
            //    robot.setRotatePos(0);
            }
            if (gamepad2.dpad_right) {
            //    robot.setRotatePos(1940);
            }

            robot.updateTarget();


            robot.setRotateSpeed(gamepad2.right_stick_y);


            robot.setLiftSpeed(gamepad2.left_stick_y);


            if (robot.getColor(-1) < 1 && gamepad2.right_trigger - gamepad2.left_trigger > 0.25){
                robot.setIntakeSpeed(0);
                robot.closeClaw();
            }
            else {
                robot.setIntakeSpeed(gamepad2.right_trigger - gamepad2.left_trigger);
            }

            if (gamepad1.a) {
                robot.setCarSpeed(1);
            }
            else {
                robot.setCarSpeed(0);
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            //position
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            //motor encoders
            telemetry.addData("rotate encoder: ", robot.rotate.getCurrentPosition());
            telemetry.addData("lift: ", robot.lift.getCurrentPosition());
            telemetry.addData("intake: ", robot.getIntakePos());

            //color sensor
            telemetry.addData("distance ", robot.getColor(-1));
            telemetry.addData("alpha ", robot.getColor(0));
            telemetry.addData("red ", robot.getColor(1));
            telemetry.addData("green ", robot.getColor(2));
            telemetry.addData("blue ", robot.getColor(3));
            telemetry.update();
        }
    }
}