package org.firstinspires.ftc.teamcode.Cogintilities;

import com.pedropathing.geometry.Pose;

public class MirrorUtility{

    public static Pose mirror(Pose bluePose) {
        double redX = 144 - bluePose.getX();
        double redY = bluePose.getY();

        double redHeading = bluePose.getHeading() + Math.toRadians(180);

        //Make heading between -180 and 180
        redHeading = Math.atan2(Math.sin(redHeading), Math.cos(redHeading));

        return new Pose(redX, redY, redHeading);
    }
}
