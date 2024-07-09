package com.team5817.frc.Field;


import java.util.HashMap;


import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;







public class FieldLayout {


    public static class Red {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();

        private static final Pose2d kTag1ToCenterAlign = new Pose2d(-3, 0.0, Rotation2d.fromDegrees(180));
        
        private static final Pose2d kTag2ToCenterAlign = new Pose2d(0.77, 0.0, Rotation2d.fromDegrees(180));
       
        private static final Pose2d kTag3ToCenterAlign = new Pose2d(-3, 0.0, Rotation2d.fromDegrees(0));
        
        private static final Pose2d kTag4ToCenterAlign = new Pose2d(-1.4, 0.0, Rotation2d.fromDegrees(0));
        
        private static final Pose2d kTag5ToCenterAlign = new Pose2d(0, -0.5, Rotation2d.fromDegrees(-90));
        

        public static final AprilTag kAprilTag1 = new AprilTag(
                1,
                1.355852,
                new Pose2d(
                        new Translation2d(15.079471999999997,0.24587199999999998),
                        Rotation2d.fromDegrees(0)
                ),
                false,
                kTag1ToCenterAlign,
                kTag1ToCenterAlign,
                kTag1ToCenterAlign
        );


        public static final AprilTag kAprilTag2 = new AprilTag(
                2,
                1.355852,
                new Pose2d(
                        new Translation2d(16.185134,0.883666),
                        Rotation2d.fromDegrees(0)
                ),
                false,
                kTag2ToCenterAlign,
                kTag2ToCenterAlign,
                kTag2ToCenterAlign
        );


        public static final AprilTag kAprilTag3 = new AprilTag(
                3,
                1.4511020000000001,
                new Pose2d(
                        new Translation2d(16.579342,4.982717999999999),
                        Rotation2d.fromDegrees(0)
                ),
                false,
                kTag3ToCenterAlign,
                kTag3ToCenterAlign,
                kTag3ToCenterAlign
        );


        public static final AprilTag kAprilTag4 = new AprilTag(
                4,
                1.4511020000000001,
                new Pose2d(
                        new Translation2d(16.579342,5.547867999999999),
                        Rotation2d.fromDegrees(0)
                ),
                false,
                kTag4ToCenterAlign,
                kTag4ToCenterAlign,
                kTag4ToCenterAlign
        );


        public static final AprilTag kAprilTag5 = new AprilTag(
                    5,
                    1.355852,
                    new Pose2d(
                            new Translation2d(14.700757999999999,8.2042),
                            Rotation2d.fromDegrees(0)
                    ),
                    true,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag6 = new AprilTag(
                    6,
                    1.355852,
                    new Pose2d(
                            new Translation2d(1.8415,8.2042),
                            Rotation2d.fromDegrees(0)
                    ),
                    true,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag7 = new AprilTag(
                    7,
                    1.4511020000000001,
                    new Pose2d(
                            new Translation2d(-0.038099999999999995,5.547867999999999),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag8 = new AprilTag(
                    8,
                    1.4511020000000001,
                    new Pose2d(
                            new Translation2d(-0.038099999999999995,4.982717999999999),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );

        public static final AprilTag kAprilTag9 = new AprilTag(
                    9,
                    1.355852,
                    new Pose2d(
                            new Translation2d(0.356108,0.883666),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag10 = new AprilTag(
                    10,
                    1.355852,
                    new Pose2d(
                            new Translation2d(1.4615159999999998,0.24587199999999998),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag11 = new AprilTag(
                    11,
                    1.3208,
                    new Pose2d(
                            new Translation2d(11.904726,3.7132259999999997),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag12 = new AprilTag(
                    12,
                    1.3208,
                    new Pose2d(
                            new Translation2d( 11.904726,4.49834),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag13 = new AprilTag(
                    13,
                    1.3208,
                    new Pose2d(
                            new Translation2d(11.220196,4.105148),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );
        public static final AprilTag kAprilTag14 = new AprilTag(
                    14,
                    1.3208,
                    new Pose2d(
                            new Translation2d(5.320792,4.105148),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );

        public static final AprilTag kAprilTag15 = new AprilTag(
                    15,
                    1.3208,
                    new Pose2d(
                            new Translation2d(4.641342,4.49834),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );

        public static final AprilTag kAprilTag16 = new AprilTag(
                    16,
                    1.3208,
                    new Pose2d(
                            new Translation2d(4.641342,3.7132259999999997),
                            Rotation2d.fromDegrees(0)
                    ),
                    false,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );

        static {
 
        kAprilTagMap.put(3, kAprilTag3);
        kAprilTagMap.put(4, kAprilTag4);
        kAprilTagMap.put(5, kAprilTag5);
        kAprilTagMap.put(7, kAprilTag7);
        kAprilTagMap.put(8, kAprilTag8);
        

        }
    }
}

