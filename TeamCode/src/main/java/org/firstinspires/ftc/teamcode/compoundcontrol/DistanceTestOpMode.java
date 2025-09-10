package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// This is an opmode that reports the values of a color sensor's distance data
@TeleOp(name = "Distance Test OpMode", group = "Linear OpMode")
public class DistanceTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
            double distance = colorSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", distance);
            telemetry.update();
        }
    }
}
