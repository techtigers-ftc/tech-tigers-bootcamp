package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// This is an opmode that reports the values of a color sensor's color data
@TeleOp(name = "Color Test OpMode", group = "Linear OpMode")
public class ColorTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
            double red = colorSensor.red();
            double green = colorSensor.green();
            double blue = colorSensor.blue();
            double alpha = colorSensor.alpha();

            telemetry.addData("Red value", red);
            telemetry.addData("Green value", green);
            telemetry.addData("Blue value", blue);
            telemetry.addData("Alpha value", alpha);
            telemetry.addLine();

            telemetry.update();
        }
    }
}
