package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveCoastOutRequest implements SwerveRequest {
	/** Local reference to a coast request for the drive motors */
	private final CoastOut m_driveRequest = new CoastOut();
	/** Local reference to a static brake request for the steer motors */
	private final StaticBrake m_steerRequest = new StaticBrake();

	@Override
	public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
		for (int i = 0; i < modulesToApply.length; ++i) {
			modulesToApply[i].apply(m_driveRequest, m_steerRequest);
		}
		return StatusCode.OK;
	}
}
