package config.auto;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class BomboclatPaths {

        public static PathBuilder builder = new PathBuilder();

        public static PathChain line1 = builder
                .addPath(
                        new BezierLine(
                                new Point(7.000, 54.500, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        public static PathChain line2 = builder
                .addPath(
                        new BezierCurve(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(30.000, 30.000, Point.CARTESIAN),
                                new Point(52.000, 35.000, Point.CARTESIAN),
                                new Point(65.000, 23.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line3 = builder
                .addPath(
                        new BezierLine(
                                new Point(65.000, 23.500, Point.CARTESIAN),
                                new Point(15.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line4 = builder
                .addPath(
                        new BezierCurve(
                                new Point(15.000, 24.000, Point.CARTESIAN),
                                new Point(55.000, 32.000, Point.CARTESIAN),
                                new Point(65.000, 12.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line5 = builder
                .addPath(
                        new BezierLine(
                                new Point(65.000, 12.500, Point.CARTESIAN),
                                new Point(15.000, 12.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line6 = builder
                .addPath(
                        new BezierCurve(
                                new Point(15.000, 12.500, Point.CARTESIAN),
                                new Point(55.000, 32.000, Point.CARTESIAN),
                                new Point(65.000, 7.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line7 = builder
                .addPath(
                        new BezierLine(
                                new Point(65.000, 7.000, Point.CARTESIAN),
                                new Point(15.000, 7.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line8 = builder
                .addPath(
                        new BezierCurve(
                                new Point(15.000, 7.000, Point.CARTESIAN),
                                new Point(35.000, 34.000, Point.CARTESIAN),
                                new Point(12.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        public static PathChain line9 = builder
                .addPath(
                        new BezierLine(
                                new Point(12.000, 30.000, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        public static PathChain line10 = builder
                .addPath(
                        new BezierLine(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(12.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line11 = builder
                .addPath(
                        new BezierLine(
                                new Point(12.000, 30.000, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line12 = builder
                .addPath(
                        new BezierLine(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(12.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line13 = builder
                .addPath(
                        new BezierLine(
                                new Point(12.000, 30.000, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line14 = builder
                .addPath(
                        new BezierLine(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(12.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line15 = builder
                .addPath(
                        new BezierLine(
                                new Point(12.000, 30.000, Point.CARTESIAN),
                                new Point(36.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line16 = builder
                .addPath(
                        new BezierCurve(
                                new Point(36.000, 65.000, Point.CARTESIAN),
                                new Point(18.000, 67.000, Point.CARTESIAN),
                                new Point(14.000, 28.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
}
