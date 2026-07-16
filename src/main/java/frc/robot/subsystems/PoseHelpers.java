package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseHelpers {

    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
    private final int[] RedHubIds  = new int[]{2, 5, 8, 9, 10, 11};
    private final int[] BlueHubIds = new int[]{18, 21, 24, 25, 26, 27};
    private final int[] RedPassIds = new int[]{9, 10, 15, 16};
    private final int[] BluePassIds  = new int[]{25, 26, 31, 32};


    public PoseHelpers(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Optional<Translation2d> getHubPosition() {

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        int[] validIds = alliance == Alliance.Red ? RedHubIds : BlueHubIds;

        double totalX = 0;
        double totalY = 0;
        int count = 0;

        for (int id : validIds) {
            var pose = fieldLayout.getTagPose(id);
            if (pose.isPresent()) {
                Translation2d t = pose.get().toPose2d().getTranslation();
                totalX += t.getX();
                totalY += t.getY();
                count++;
            }
        }

        if (count == 0)
            return Optional.empty();

        return Optional.of(new Translation2d(totalX / count, totalY / count));
    }
    
    public Optional<Rotation2d> getHubHeading() {
        
        Optional<Translation2d> hubPosOpt = getHubPosition();
        if (hubPosOpt.isEmpty())
            return Optional.empty();

        Pose2d robotPose = drivetrain.getState().Pose;

        Translation2d delta = hubPosOpt.get().minus(robotPose.getTranslation());

        return Optional.of(delta.getAngle());
    }

    public Optional<Double> getDistanceToHub() {

        Optional<Translation2d> hubPosOpt = getHubPosition();
        if (hubPosOpt.isEmpty())
            return Optional.empty();

        Pose2d robotPose = drivetrain.getState().Pose;
        
        return Optional.of(
            hubPosOpt.get()
            .getDistance(robotPose.getTranslation())
            );
        }
        
    public Optional<Translation2d> getPassTargetPosition() {

        Alliance alliance =
            DriverStation.getAlliance().orElse(Alliance.Red);
        int[] validIds =
            alliance == Alliance.Red ? RedPassIds : BluePassIds;
        Translation2d sum = new Translation2d();
        int count = 0;
        for (int id : validIds) {
            var pose = fieldLayout.getTagPose(id);
            if (pose.isPresent()) {
                sum = sum.plus(
                    pose.get().toPose2d().getTranslation()
                );
                count++;
            }
        }
        if (count == 0) return Optional.empty();
        return Optional.of(sum.div(count));
    }

    public Optional<Rotation2d> getPassHeading() {
        Optional<Translation2d> targetOpt =
            getPassTargetPosition();
        if (targetOpt.isEmpty()) return Optional.empty();
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d delta =
            targetOpt.get()
                .minus(robotPose.getTranslation());
        return Optional.of(delta.getAngle());
    }
}