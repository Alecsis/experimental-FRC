package frc.robot.auto;

/**
 * Golden regression metadata for one auto, stored as JSON under
 * src/test/resources/autoRegression/. Regression signal only -- completed/runtime/max-tracking-
 * error are asserted against; the diagnostic poses are informational only and never compared.
 * See docs/superpowers/specs/2026-07-18-auto-regression-suite-design.md.
 *
 * <p>Public fields, no getters/setters: Jackson's default field visibility picks these up as-is,
 * and this is a plain data holder read/written wholesale, never partially mutated.
 */
class AutoRegressionGolden {
    public String autoName;
    public String recordedAtGitSha;
    public boolean completed;
    public double runtimeSeconds;
    public double maxLateralErrorMeters;
    public double maxLongitudinalErrorMeters;
    public DiagnosticPose diagnosticStartPose;
    public DiagnosticPose diagnosticFinalPose;

    AutoRegressionGolden() {}

    static class DiagnosticPose {
        public double xMeters;
        public double yMeters;
        public double thetaRadians;

        DiagnosticPose() {}

        DiagnosticPose(double xMeters, double yMeters, double thetaRadians) {
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.thetaRadians = thetaRadians;
        }
    }
}
