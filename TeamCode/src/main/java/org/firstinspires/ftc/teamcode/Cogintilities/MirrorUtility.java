package org.firstinspires.ftc.teamcode.Cogintilities;

import com.pedropathing.geometry.Pose;

public class MirrorUtility{

    private MirrorUtility(){}

    public static Pose mirror(Pose bluePose) {
        double redX = 144 - bluePose.getX();
        double redY = bluePose.getY();

        double redHeading = Math.toRadians(180) - bluePose.getHeading();

        //Make heading between -180 and 180
        redHeading = Math.atan2(Math.sin(redHeading), Math.cos(redHeading));

        return new Pose(redX, redY, redHeading);
    }
}
