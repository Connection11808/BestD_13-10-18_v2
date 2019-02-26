package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ConnectionTp", group="Connection")

public class ConnectionTp extends LinearOpMode {
    public void runOpMode() {
        /* Declare OpMode members. */
        Hardware_Connection robot = new Hardware_Connection();
        double armPower;
        double openingPower;
        double collectingPower;

        robot.init(hardwareMap);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData(".", "done init ");
        telemetry.update();
        waitForStart();
        robot.mineral_keeper_servo.setPosition(0.1);

        while (opModeIsActive()) {
            armPower = -gamepad2.left_stick_y;
            openingPower = gamepad2.right_stick_y;
            collectingPower = gamepad1.right_trigger - gamepad1.left_trigger;

            if(gamepad1.dpad_left){
                robot.team_marker_servo.setPosition(0);
            }
            else if(gamepad1.dpad_right){
                robot.team_marker_servo.setPosition(1);
            }

            if (armPower < -0.6) {
                armPower = -0.6;
            }
            if (armPower > 0.6) {
                    armPower = 0.6;
            }

            robot.arm_motors(armPower);
            telemetry.addData("ArmPower", armPower);

            robot.arm_opening_system.setPower(openingPower);
            telemetry.addData("OpeningPower", openingPower);

            if (gamepad1.dpad_down) {
                robot.mineral_keeper_servo.setPosition(1);
            }
            if (!gamepad1.dpad_down) {
                robot.mineral_keeper_servo.setPosition(0);
            }
            if (gamepad1.y) {
                robot.climbing_motors();
            }


            robot.arm_collecting_system.setPower(collectingPower);

            String left_stick_quarter = robot.whichQuarter(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.3);
            String right_stick_quarter = robot.whichQuarter(-gamepad1.right_stick_y, gamepad1.right_stick_x, 0.3);
            telemetry.addData("right stick quarter", right_stick_quarter);
            telemetry.addData("left stick quarter", left_stick_quarter);
            telemetry.addData("power 1",gamepad1.right_stick_x);
            telemetry.addData("power 2",gamepad1.right_stick_y);
            telemetry.addData("power 3", gamepad1.left_stick_x);
            telemetry.addData("power 4", gamepad1.left_stick_y);
            telemetry.addData("collectingPower value", collectingPower);

            switch (left_stick_quarter) {
                case "unusedZone":
                    robot.leftDriveY(0, 0);
                    break;
                case "up":
                    robot.leftDriveY(-gamepad1.left_stick_y, 0.9);
                    break;
                case "right":
                    robot.leftDriveX(-gamepad1.left_stick_x, 0.9);
                    break;
                case "down":
                    robot.leftDriveY(-gamepad1.left_stick_y, 0.9);
                    break;
                case "left":
                    robot.leftDriveX(-gamepad1.left_stick_x, 0.9);
                    break;
                default:
                    robot.leftDriveY(0, 0);
                    break;
            }


            switch (right_stick_quarter) {
                case "unusedzone":
                    robot.rightDriveY(0, 0);
                    break;
                case "up":
                    robot.rightDriveY(-gamepad1.right_stick_y, 0.9);
                    break;
                case "right":
                    robot.rightDriveX(-gamepad1.right_stick_x, 0.9);
                    break;
                case "down":
                    robot.rightDriveY(-gamepad1.right_stick_y, 0.9);
                    break;
                case "left":
                    robot.rightDriveX(-gamepad1.right_stick_x, 0.9);
                    break;
                default:
                    robot.rightDriveY(0, 0);

                    break;
            }
            telemetry.update();
        }
    }
}
