package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Crater", group="Detroit")

public class AutoCraterDetroit extends functions {


    public void runOpMode() {
        InitRobot();
        if (opModeIsActive()) {
            auto.Detroit(true);            
        }
    }


}