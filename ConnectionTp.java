package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ConnectionTp", group="Connection")

public class ConnectionTp extends LinearOpMode {
    public void runOpMode() {
        /* Declare OpMode members. */
        Hardware_Connection robot = new Hardware_Connection();
        double armPower, maxSpeed = 0.7;

        robot.init(hardwareMap);
        telemetry.addData(".", "done init ");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {


            //set the value from the trigger on "armPower".
            armPower = gamepad2.left_trigger - gamepad2.right_trigger;
            //check if the "armPower" value is higher than 0.5 and if it does the value become 0.5.
            if (armPower > 0.5) {
                armPower = 0.5;
            }
            //gives the arm motor the value of the "armPower".
            robot.arm_motors(armPower * maxSpeed);

            //checks if both triggers are pressed and if so the arm motor value will be 0.
            if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
                //turn the power off.
                robot.arm_motor_1.setPower(0);
            }

            //check if the left bumper pressed.
            if (gamepad2.left_bumper) {
                //and if it does, sets the power to 1.
                robot.arm_opening_system.setPower(1);
            }

            //checks if the right bumper is pressed.
            else if (gamepad2.right_bumper) {
                //and if it does, sets the power to -1.
                robot.arm_opening_system.setPower(-1);
            }

            //checks if both bumpers are pressed.
            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                //and if it does, the openning system's power will be 0.
                robot.arm_opening_system.setPower(0);
            }

            //adds the value of the joysticks to the "Driver Station".
            telemetry.addData("rightJoystickY value", gamepad1.right_stick_y);
            telemetry.addData("rightJoystickX value", gamepad1.right_stick_x);
            telemetry.addData("leftJoystickY value", gamepad1.left_stick_y);
            telemetry.addData("leftJoystickX value", gamepad1.right_stick_x);
            telemetry.update();

            String left_stick_quarter = robot.whichQuarter(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.2);
            String right_stick_quarter = robot.whichQuarter(-gamepad1.right_stick_y, gamepad1.right_stick_x, 0.2);
            String right_stick_diagonl_quarter = robot.whichDiagonalQuarter(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.2);
            String left_stick_diagonl_quarter = robot.whichDiagonalQuarter(-gamepad1.right_stick_y, gamepad1.right_stick_x, 0.2);

            if (gamepad2.a) {
                robot.arm_collecting_system.setPower(-0.7);
            } else if (gamepad2.x) {
                robot.arm_collecting_system.setPower(0.7);
            } else {
                robot.arm_collecting_system.setPower(0);
            }
            if (gamepad2.y) {
                robot.climbing_motors();
            }
            switch (left_stick_quarter) {
                case "unusedzone":
                    robot.leftDriveY(0, 0);
                    break;
                case "up":
                    robot.leftDriveY(-gamepad1.left_stick_y, 1);
                    break;
                case "right":
                    robot.leftDriveX(-gamepad1.left_stick_x, 1);
                    break;
                case "down":
                    robot.leftDriveY(-gamepad1.left_stick_y, 1);
                    break;
                case "left":
                    robot.leftDriveX(-gamepad1.left_stick_x, 1);
                    break;
                default:
                    robot.leftDriveY(0, 0);
                    telemetry.addData("ERROR", "left_stick_quarter");
                    telemetry.update();
                    break;
            }

            switch (right_stick_quarter) {
                case "unusedzone":
                    robot.rightDriveY(0, 0);
                    break;
                case "up":
                    robot.rightDriveY(-gamepad1.right_stick_y, 1);
                    break;
                case "right":
                    robot.rightDriveX(-gamepad1.right_stick_x, 1);
                    break;
                case "down":
                    robot.rightDriveY(-gamepad1.right_stick_y, 1);
                    break;
                case "left":
                    robot.rightDriveX(-gamepad1.right_stick_x, 1);
                    break;
                default:
                    robot.rightDriveY(0, 0);
                    telemetry.addData("ERROR", "right_stick_quarter");
                    telemetry.update();
                    break;
            }
           /* switch (right_stick_quarter) {
                case "unusedzone":
                    robot.diagonalDriveRight(0,0);
                    break;
                case "leftFront":
                    robot.diagonalDriveRight(-gamepad1.right_stick_y - -gamepad1.right_stick_x,1);
                    break;
                case "rightBack":
                    robot.diagonalDriveRight(-gamepad1.right_stick_x - -gamepad1.right_stick_y, 1);
                    break;
                case "leftBack":
                    robot.diagonalDriveRight(-gamepad1.right_stick_y -);

            }switch (left_stick_quarter) {
                case "unusedzone":
                    robot.diagonalDriveRight(0,0);
                    break;
                case "leftFront":
                    robot.diagonalDriveRight(-gamepad1.right_stick_y - -gamepad1.right_stick_x,1);
                    break;
                case "rightBack":
                    robot.diagonalDriveRight(-gamepad1.right_stick_x - -gamepad1.right_stick_y, 1);
                    break;

            }

        }*/
        }
    }
}
