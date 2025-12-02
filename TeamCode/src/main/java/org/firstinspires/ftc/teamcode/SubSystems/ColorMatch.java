package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

public class ColorMatch implements TeamConstants {

    RevColorSensorV3 colorSensor;


    public ColorMatch(RevColorSensorV3 colorSensor) {
        this.colorSensor = colorSensor;
    }

    //Get the raw RGB Values
    public int[] myRGB() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        return new int[]{r, g, b};
    }

    //use library function to convert RGB to hue, saturation, and value
    public float[] getHSV() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        return hsv;
    }

    //Detect the color
    public String detectColor() {
        float[] hsv = getHSV();
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < 0.2 || val < 0.2) return "UNKNOWN";

        String detectedColor = "UNKNOWN";

            if (hue < 30 || hue > 330) {
                detectedColor = "RED";
            } else if (hue > 90 && hue < 150) {
                detectedColor = "GREEN";
            } else if (hue > 200 && hue < 270) {
                detectedColor = "BLUE";
            } else if (hue > 220 && hue < 300)
                detectedColor = "PURPLE";

        return detectedColor;
    }

}
