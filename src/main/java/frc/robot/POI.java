package frc.robot;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public enum POI {
    Hub(
    new Translation2d(Meters.of(2.28),Meters.of(3.25)), //blye
    new Translation2d(Meters.of(11.915394),Meters.of(4.0132)), //red
    Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(180)
    ),

        LeftStage(
    new Translation2d(Meters.of(2.036),Meters.of(5.799)), //blye
    new Translation2d(Meters.of(14.622),Meters.of(2.297)),//red
    Rotation2d.fromDegrees(-36),
    Rotation2d.fromDegrees(146)    ),
        CenterStage(
    new Translation2d(Meters.of(2.162),Meters.of(4.093)), //blye
    new Translation2d(Meters.of(14.285),Meters.of(4.123)),//red
    Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(180)    ),
        RightStage(
    new Translation2d(Meters.of(2.244),Meters.of(1.990)), //blye
    new Translation2d(Meters.of(14.794),Meters.of(6.314)),//red
    Rotation2d.fromDegrees(46),
    Rotation2d.fromDegrees(-138)    ),
    Right(
    new Translation2d(Meters.of(7.708),Meters.of(0.714)), //blye
    new Translation2d(Meters.of(7.708),Meters.of(7.29)),//red
    Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(180)    ),
    Left(
    new Translation2d(Meters.of(7.708),Meters.of(7.29)), //blye
    new Translation2d(Meters.of(7.708),Meters.of(0.714)),//red
    Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(180)        
    );
    private final Translation2d bluePose;
    private final Translation2d redPose;
    private final Rotation2d blueRot;
    private final Rotation2d redRot;
    POI(Translation2d bluePose, Translation2d redPose, Rotation2d blueRot, Rotation2d redRot) {
        this.bluePose = bluePose;
        this.redPose = redPose;
        this.blueRot = blueRot;
        this.redRot = redRot;
    }
    public Translation2d get() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return redPose;
        }
        return bluePose;
    }
    public Rotation2d getTargetRotation() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return redRot;
        }
        return blueRot;
    }
}