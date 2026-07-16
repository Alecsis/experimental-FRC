package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Target extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final java.util.function.Supplier<Optional<Translation2d>> targetSupplier;
    private final double maxSpeed;
    private final PIDController headingPID = new PIDController(5.0, 0.0, 0.15);
    private final java.util.function.DoubleSupplier xSupplier;
    private final java.util.function.DoubleSupplier ySupplier;
    private static final double kHeadingToleranceRad = Math.toRadians(2.0);
    private final boolean finishOnAlign;
    private final SwerveRequest.FieldCentric driveReqent = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.Idle idleReq = new SwerveRequest.Idle();

    public Target(
            CommandSwerveDrivetrain drivetrain,
            java.util.function.Supplier<Optional<Translation2d>> targetSupplier,
            double maxSpeed,
            java.util.function.DoubleSupplier xSupplier,
            java.util.function.DoubleSupplier ySupplier,
            boolean finishOnAlign) {
        this.drivetrain = drivetrain;
        this.targetSupplier = targetSupplier;
        this.maxSpeed = maxSpeed;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.finishOnAlign = finishOnAlign;
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    private double angle(Translation2d hubPos, Pose2d robotPose) {
        double offsetDeg = SmartDashboard.getNumber("offset", 0);
        Translation2d toHub = hubPos.minus(robotPose.getTranslation());
        return toHub.getAngle().getRadians() + Math.toRadians(offsetDeg);
    }
    @Override
    public void initialize() {
        headingPID.reset();
        double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        headingPID.calculate(currentHeading, currentHeading);
    }

    @Override
    public void execute() {

        Optional<Translation2d> hubPosOpt = targetSupplier.get();

        if (hubPosOpt.isEmpty()) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        Pose2d robotPose = drivetrain.getState().Pose;
        double target = angle(hubPosOpt.get(), robotPose);
        double rotationOutput = headingPID.calculate(
                robotPose.getRotation().getRadians(),
                target);

        double vx = -ySupplier.getAsDouble() * maxSpeed;
        double vy = -xSupplier.getAsDouble() * maxSpeed;

        drivetrain.setControl(
                driveReqent
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(rotationOutput));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(idleReq);
    }

    @Override
    public boolean isFinished() {
        if (!finishOnAlign)
            return false;

        Optional<Translation2d> hubPosOpt = targetSupplier.get();
        if (hubPosOpt.isEmpty())
            return false;

        Pose2d robotPose = drivetrain.getState().Pose;
        double target = angle(hubPosOpt.get(), robotPose);
        double error = Math.abs(
                edu.wpi.first.math.MathUtil.angleModulus(
                        robotPose.getRotation().getRadians() - target));
        return error < kHeadingToleranceRad;
    }
}