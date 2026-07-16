package frc.lib.util.vision;

public record CameraPipeline(CameraPipelineType type, String name, int index) {

	public static CameraPipeline getDefault() {
		return new CameraPipeline(CameraPipelineType.DISABLED, "DEFAULT", -1);
	}
}
