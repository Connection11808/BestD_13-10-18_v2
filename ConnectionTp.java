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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ConnectionTp", group="Connection")

public class ConnectionTp extends LinearOpMode {
    public void runOpMode() {
        /* Declare OpMode members. */
        Hardware_Connection robot = new Hardware_Connection();
        double armPower, maxSpeed = 1;

        robot.init(hardwareMap);
        telemetry.addData(".", "done init ");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //adds the value of the joysticks to the "Driver Station".
            telemetry.addData("rightJoystickY value", gamepad1.right_stick_y);
            telemetry.addData("leftJoystickY value", gamepad1.left_stick_y);
            telemetry.update();

            //set the value from the trigger on "armPower".
            armPower = gamepad2.left_trigger - gamepad2.right_trigger;
            //check if the "armPower" value is higher than 0.5 and if it does the value become 0.5.
            if (armPower > 0.5) {
                armPower = 0.5;
            }
            //gives the arm motor the value of the "armPower".
            robot.arm_motor_2.setPower((armPower) * maxSpeed);

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

            String left_quarter_straight = robot.whichStraightQuarter(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.2);
            String right_quarter_straight = robot.whichStraightQuarter(-gamepad1.right_stick_y, gamepad1.right_stick_x, 0.2);

            switch (left_quarter_straight) {
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
            }

            switch (right_quarter_straight) {
                case "unusedzone":
                    robot.rightDriveX(0, 0);
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
            }
        }
    }
}
