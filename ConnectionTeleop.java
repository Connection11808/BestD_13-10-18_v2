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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ConnectionTeleop", group="Connection")

public class ConnectionTeleop extends OpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();
    double left_speed;
    double right_speed;
    double arm_power;
    double arm_power_REVERSE;


    @Override
    public void init() {
        /** Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

    }


    @Override
    public void loop() {

        //makes the robot's "arms" able to be controlled with the triggers.
        arm_power = gamepad2.right_trigger;
        arm_power_REVERSE = gamepad2.left_trigger;
        left_speed = -gamepad1.left_stick_y;
        right_speed = -gamepad1.right_stick_y;
        telemetry.addData("right drive value:", right_speed);
        telemetry.addData("left drive value:", left_speed);
        telemetry.update();

        robot.fullDriving(left_speed, right_speed);

        arm_power = gamepad2.right_trigger;
        arm_power_REVERSE = gamepad2.left_trigger;

        robot.arm_motors(arm_power);
        robot.arm_motors_REVERSE(arm_power_REVERSE);
        // makes the robot's "arm opening system" able to be controlled with the bumpers.
        if (gamepad2.right_bumper) {
            robot.arm_opening_system.setPower(1);
        }
        if (gamepad2.left_bumper) {
            robot.arm_opening_system.setPower(-1);
        }
        robot.arm_opening_system.setPower(0);

        if (gamepad2.b) {
            robot.arm_collecting_system.setPower(1);
        }
        if (gamepad2.a) {
            robot.arm_collecting_system.setPower(-1);
        }
        if (!gamepad2.a && !gamepad2.b){
            robot.arm_collecting_system.setPower(0);
        }

    }

}
