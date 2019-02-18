package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ConnectionTp", group="Connection")

public class ConnectionTp extends LinearOpMode {
    public void runOpMode() {
        /* Declare OpMode members. */
        Hardware_Connection robot = new Hardware_Connection();
        double armPower;
        double openingPower;
        double collectingPower;

        robot.init(hardwareMap);
        telemetry.addData(".", "done init ");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {


            armPower = gamepad2.left_stick_y;
            if (armPower <= -0.6 ){
                armPower = -0.6;
            }
            if (armPower > 0.6 ){
                armPower = 0.6;
            }

            robot.arm_motors(armPower);
            openingPower = gamepad2.right_stick_y;
            if (openingPower > 0){
                openingPower = 1;
            }
            if (openingPower < 0){
                openingPower = -1;
            }
            robot.arm_opening_system.setPower(openingPower);

            collectingPower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.arm_collecting_system.setPower(collectingPower);

            //checks if both bumpers are pressed.
            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                //and if it does, the openning system's power will be 0.
                robot.arm_opening_system.setPower(0);
            }

            //adds the value of the joysticks to the "Driver Station".
            telemetry.addData("armPower value",armPower);
            telemetry.addData("openingPower value",openingPower);
            telemetry.addData("collectingPower value",collectingPower);
            telemetry.addData("ARM power vale", armPower);
            telemetry.update();

            String left_stick_quarter = robot.whichQuarter(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.2);
            String right_stick_quarter = robot.whichQuarter(-gamepad1.right_stick_y, gamepad1.right_stick_x, 0.2);

            if (gamepad2.a) {
                robot.arm_collecting_system.setPower(-0.6);
            }
            else if (gamepad2.x) {
                robot.arm_collecting_system.setPower(0.6);
            }
            else {
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
                    telemetry.addData("ERROR", "left_stick_quarter");
                    telemetry.update();
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
                    telemetry.addData("ERROR", "right_stick_quarter");
                    telemetry.update();
                    break;
            }
        }


    }
}

