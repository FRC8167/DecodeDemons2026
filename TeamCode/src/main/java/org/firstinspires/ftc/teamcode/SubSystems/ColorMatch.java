package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorColor;

import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

public class ColorMatch extends SubsystemBase {

SensorColor sensorColor
        ;


    public ColorMatch(SensorColor sensorColor) {
        this.sensorColor = sensorColor;
    }

    //Get the raw RGB Values
    public int[] myRGB() {
        int r = sensorColor.red();
        int g = sensorColor.green();
        int b = sensorColor.blue();
        return new int[]{r, g, b};
    }

    //use library function to convert RGB to hue, saturation, and value
    public float[] getHSV() {
        int r = sensorColor.red();
        int g = sensorColor.green();
        int b = sensorColor.blue();
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
            } else if (hue > 70 && hue < 160  && hue!=120) {
                detectedColor = "GREEN";
            } else if (hue > 220 && hue < 300)
                detectedColor = "PURPLE";

        return detectedColor;
    }

}
