package config.auto;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class FiveSpecPaths {

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(7.000, 54.000, Point.CARTESIAN),
                            new Point(35.000, 62.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierCurve(
                            new Point(35.000, 62.000, Point.CARTESIAN),
                            new Point(29.419, 22.452, Point.CARTESIAN),
                            new Point(58.323, 52.129, Point.CARTESIAN),
                            new Point(58.000, 25.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(58.000, 25.000, Point.CARTESIAN),
                            new Point(12.000, 25.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierCurve(
                            new Point(12.000, 25.000, Point.CARTESIAN),
                            new Point(59.613, 26.323, Point.CARTESIAN),
                            new Point(55.226, 27.613, Point.CARTESIAN),
                            new Point(57.000, 13.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierLine(
                            new Point(57.000, 13.000, Point.CARTESIAN),
                            new Point(12.000, 13.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line6 = builder
            .addPath(
                    new BezierCurve(
                            new Point(12.000, 13.000, Point.CARTESIAN),
                            new Point(32.000, 11.871, Point.CARTESIAN),
                            new Point(59.100, 18.050, Point.CARTESIAN),
                            new Point(56.516, 6.125, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line7 = builder
            .addPath(
                    new BezierLine(
                            new Point(56.516, 6.125, Point.CARTESIAN),
                            new Point(12.000, 6.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line8 = builder
            .addPath(
                    new BezierLine(
                            new Point(12.000, 6.000, Point.CARTESIAN),
                            new Point(12.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
            .build();

    public static PathChain line9 = builder
            .addPath(
                    new BezierCurve(
                            new Point(12.000, 30.000, Point.CARTESIAN),
                            new Point(12.903, 62.000, Point.CARTESIAN),
                            new Point(40.000, 62.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain line10 = builder
            .addPath(
                    new BezierCurve(
                            new Point(40.000, 62.000, Point.CARTESIAN),
                            new Point(12.903, 62.000, Point.CARTESIAN),
                            new Point(12.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();
}