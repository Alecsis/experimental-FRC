package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.POI;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignPOI extends Command {

    private static final double kTolerance = 0.20; //tolerance
    private static final double kRotationTolerance = Math.toRadians(1); //tolerance
    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric request =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController x = new PIDController(1, 0, 0);
    private final PIDController y = new PIDController(1, 0, 0);
    private final PIDController rot = new PIDController(3, 0, 0);
    private static final double kMaxSpeed = .5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // robotcontainer 
    private static final double kMaxRotationalSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // robotcontainer 
    private final POI targetPOI;
    
    public AutoAlignPOI(CommandSwerveDrivetrain swerve, POI targetPOI) {
        this.swerve = swerve;
        this.targetPOI = targetPOI;

        x.setTolerance(kTolerance);
        y.setTolerance(kTolerance);
        rot.setTolerance(kRotationTolerance);
        rot.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(swerve);
    }
    //turn
    private Translation2d robotPos() {
        return swerve.getState().Pose.getTranslation();
    }
    //turn but angle
    private Rotation2d rotRobot(){
        return swerve.getState().Pose.getRotation();
    }

    @Override
    public void execute() {
        Translation2d target = targetPOI.get();
        Translation2d current = robotPos();
        Translation2d vectorToHub = target.minus(current);
        //blackmagic voodoo shit
        Translation2d opVector = new Translation2d(
            vectorToHub.getX() * swerve.getOperatorForwardDirection().getCos() +
            vectorToHub.getY() * swerve.getOperatorForwardDirection().getSin(),
            -vectorToHub.getX() * swerve.getOperatorForwardDirection().getSin() +
            vectorToHub.getY() * swerve.getOperatorForwardDirection().getCos()
        );

        // drive cmd
        double vx = MathUtil.clamp(x.calculate(0, opVector.getX()), -kMaxSpeed, kMaxSpeed);
        double vy = MathUtil.clamp(y.calculate(0, opVector.getY()), -kMaxSpeed, kMaxSpeed);

        //rotation cmd
                double currentRotation = rotRobot().getRadians();
        double targetRotation = targetPOI.getTargetRotation().getRadians();
        double rotError = MathUtil.angleModulus(targetRotation - currentRotation);
        double dxRotRadiansPerSecond = rot.calculate(currentRotation, currentRotation + rotError);
        double dxRot = MathUtil.clamp(
            dxRotRadiansPerSecond / (2 * Math.PI), 
            -kMaxRotationalSpeed, 
            kMaxRotationalSpeed
        );


        swerve.setControl(
            request
                .withVelocityX(MetersPerSecond.of(vx))
                .withVelocityY(MetersPerSecond.of(vy))
                .withRotationalRate(dxRot) // could do radian per sec
        );
    }
    @Override
    public boolean isFinished() {
        boolean atPos = robotPos().getDistance(targetPOI.get()) < kTolerance;
        boolean atRot = Math.abs(MathUtil.angleModulus(
            rotRobot().getRadians() - targetPOI.getTargetRotation().getRadians()
        )) < kRotationTolerance;
        return atPos && atRot;

    }
    @Override
    public void end(boolean interrupted) {
        swerve.idle();
    }
}
