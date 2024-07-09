package com.team5817.frc.Field;

import com.team254.lib.geometry.Pose2d;

public class AprilTag {

    private int id;
    private double height;
    private Pose2d fieldToTag;
    private boolean isScoring;

    private Pose2d tagToLeftAlign;

    private Pose2d tagToRightAlign;

    private Pose2d tagToCenterAlign;

    public AprilTag(int id, double height, Pose2d fieldToTag, boolean isScoring, Pose2d tagToCenterAlign, Pose2d tagToLeftAlign, Pose2d tagToRightAlign) {
        this.id = id;
        this.height = height;
        this.fieldToTag = fieldToTag;
        this.isScoring = isScoring;
        this.tagToCenterAlign = tagToCenterAlign;
        this.tagToLeftAlign = tagToLeftAlign;
        this.tagToRightAlign = tagToRightAlign;
    }

    public int getId() {
        return id;
    }

    public double getHeight() {
        return height;
    }

    public Pose2d getFieldToTag() {
        return fieldToTag;
    }

    public boolean isScoring() {
        return isScoring;
    }

    public Pose2d getTagToCenterAlign() {
        return tagToCenterAlign;
    }

    public Pose2d getTagToLeftAlign() {
        return tagToLeftAlign;
    }

    public Pose2d getTagToRightAlign() {
        return tagToRightAlign;
    }
}
