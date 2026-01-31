    package org.firstinspires.ftc.teamcode.pedroPathing;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.BezierCurve;
    import com.pedropathing.geometry.BezierLine;
    import com.pedropathing.geometry.Pose;
    import com.pedropathing.paths.PathChain;

    /* TESTING PURPOSE, NOT FINAL */
    public class AutonPedroPathing {
        public static class Paths {
            public PathChain Path1;
            public PathChain Path2;
            public PathChain Path3;
            public PathChain Path4;
            public PathChain Path5;
            public PathChain Path6;
            public PathChain Path7;

            public Paths(Follower follower) {
                Path1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(56.000, 8.000),

                                        new Pose(56.000, 8.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))
                        .addParametricCallback(100, Runnable)
                        .build();
                Path2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(56.000, 8.000),
                                        new Pose(61.987, 39.694),
                                        new Pose(44.444, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(44.444, 36.000),

                                        new Pose(34.923, 36.000)
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(34.923, 36.000),

                                        new Pose(29.928, 36.000)
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(29.928, 36.000),

                                        new Pose(18.367, 35.943)
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(18.367, 35.943),
                                        new Pose(65.513, 40.817),
                                        new Pose(56.042, 7.824)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(105))
                        .setReversed()
                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(56.042, 7.824),

                                        new Pose(46.363, 60.029)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))

                        .build();
            }
        }
    }