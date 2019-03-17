package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;

@TeleOp(name="Connectiontp", group="Connection")

public class ConnectionTp extends LinearOpMode {
    public Hardware_Connection robot = new Hardware_Connection();

    double armPower;
    double openingPower;
    double collectingPower;
    double DriveY = 0;
    double DriveX = 0;
    int Degree = 0;
    double DrivePower = 0;
    double TurnPower = 0;



    public void runOpMode() {
        /* Declare OpMode members. */

        robot.init(hardwareMap);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Finish", "done init ");
        telemetry.update();
        waitForStart();
        robot.mineral_keeper_servo.setPosition(0.1);

        while (opModeIsActive()) {

            FreeFlowDrive();
            armMotorControl();
            mineralKeeperControl();
            autoClimbControl();
            colecttMineral();
            teamMarkerControl();
            openArmControl();

            telemetry.addData("RSX", gamepad1.right_stick_x);
            telemetry.addData("RSY", gamepad1.right_stick_y);
            telemetry.addData("RS", gamepad1.left_stick_x);
            telemetry.addData("power 4", gamepad1.left_stick_y);
            telemetry.update();
        }
    }


    public void ArmClimbPosition() {
        int Target = 2500;
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_motor_2.setTargetPosition((int) Target);
        telemetry.addData("Lift current:", robot.arm_motor_2.getCurrentPosition());
        telemetry.addData("Lift target:", Target);
        telemetry.update();
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Target > robot.arm_motor_2.getCurrentPosition()) {
            robot.arm_motors(1);
        }
        else {
            robot.arm_motors(0);
        }
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
        telemetry.addData("target1:", Target);
        telemetry.update();


    }
    public void FreeFlowDrive(){
        TurnPower = gamepad1.right_stick_x;

        DriveY = -gamepad1.left_stick_y;
        DriveX = gamepad1.left_stick_x;

        Degree = (int) Math.toDegrees(Math.atan2(DriveY,DriveX))+90;

        telemetry.addData("Angle",Degree);
        Degree+=45;

        DrivePower = Math.sqrt(Math.pow(DriveX,2) + Math.pow(DriveY,2));

        DriveY = DrivePower * Math.sin(Math.toRadians(Degree));
        DriveX = DrivePower * Math.cos(Math.toRadians(Degree));
        robot.diagonalLeft(-DriveY);
        robot.diagonalRight(-DriveX);
        if(TurnPower!=0) {
            robot.fullDriving(TurnPower, -TurnPower);
        }
        telemetry.addData("DriveX",DriveX);
        telemetry.addData("DriveY",DriveY);

    }
    public void armMotorControl(){
        armPower = -gamepad2.left_stick_y;
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());

        if(robot.arm_motor_2.getCurrentPosition()>3100 ) {
            robot.arm_motors(0);
            if (armPower > 0.3) {
                armPower = 0.3;
            }
            if (robot.arm_motor_2.getCurrentPosition() > 3900){
                if (armPower > 0){
                    armPower = 0;
                }
            }
        }

        robot.arm_motors(armPower);
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("ArmPower", armPower);

    }
    public void mineralKeeperControl(){

        if (gamepad1.dpad_down) {
            robot.mineral_keeper_servo.setPosition(0);
        } else {
            robot.mineral_keeper_servo.setPosition(0.5);
        }
    }
    public void autoClimbControl(){

        if (gamepad1.y) {
            robot.climbing_motors();
        }

        if(gamepad2.y){
            ArmClimbPosition();
        }
    }
    public void colecttMineral(){
        collectingPower = gamepad1.left_trigger - gamepad1.right_trigger;
        robot.arm_collecting_system.setPower(collectingPower);
    }
    public void teamMarkerControl(){
        if (gamepad1.dpad_left) {
            robot.team_marker_servo.setPosition(0);
        } else if (gamepad1.dpad_right) {
            robot.team_marker_servo.setPosition(0.2);
        }
    }
    public void openArmControl(){
        openingPower = gamepad2.right_stick_y;

        robot.arm_opening_system.setPower(openingPower);
        telemetry.addData("OpeningPower", openingPower);
    }

}
