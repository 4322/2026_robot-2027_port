package frc.robot.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;
import java.util.ArrayList;

/*
 * Code by FRC 4201
 * https://github.com/4201VitruvianBots/Rebuilt2026/blob/main/src/main/java/frc/team4201/lib/geometry/FieldRectangle2d.java
 */
public class FieldRectangle2d extends OrangeRectangle2d {

    public FieldRectangle2d(Pose2d center, double xWidth, double yWidth) {
        super(center, xWidth, yWidth);
    }

    public FieldRectangle2d(Pose2d center, Distance xWidth, Distance yWidth) {
        super(center, xWidth, yWidth);
    }

    public FieldRectangle2d(Translation2d cornerA, Translation2d cornerB) {
        super(cornerA, cornerB);
    }

    public Translation2d[] getCornerTranslations() {
        ArrayList<Translation2d> corners = new ArrayList<>();
        corners.add(new Translation2d(getCenter().getMeasureX().plus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().plus(getMeasureYWidth().div(2.0))));
        corners.add(new Translation2d(getCenter().getMeasureX().plus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().minus(getMeasureYWidth().div(2.0))));
        corners.add(new Translation2d(getCenter().getMeasureX().minus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().minus(getMeasureYWidth().div(2.0))));
        corners.add(new Translation2d(getCenter().getMeasureX().minus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().plus(getMeasureYWidth().div(2.0))));
        return corners.toArray(new Translation2d[0]);
    }

    public Pose2d[] getCornerPoses() {
        ArrayList<Pose2d> corners = new ArrayList<>();
        corners.add(new Pose2d(getCenter().getMeasureX().plus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().plus(getMeasureYWidth().div(2.0)), Rotation2d.kZero));
        corners.add(new Pose2d(getCenter().getMeasureX().plus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().minus(getMeasureYWidth().div(2.0)), Rotation2d.kZero));
        corners.add(new Pose2d(getCenter().getMeasureX().minus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().minus(getMeasureYWidth().div(2.0)), Rotation2d.kZero));
        corners.add(new Pose2d(getCenter().getMeasureX().minus(getMeasureXWidth().div(2.0)), getCenter().getMeasureY().plus(getMeasureYWidth().div(2.0)), Rotation2d.kZero));
        return corners.toArray(new Pose2d[0]);
    }
}
