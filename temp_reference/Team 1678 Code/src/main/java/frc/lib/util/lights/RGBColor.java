package frc.lib.util.lights;

import com.ctre.phoenix6.signals.RGBWColor;

public class RGBColor {
	public final int r;
	public final int g;
	public final int b;

	public RGBColor(int red, int green, int blue) {
		r = red;
		g = green;
		b = blue;
	}

	public static RGBColor fromWPIColor(edu.wpi.first.wpilibj.util.Color color) {
		return new RGBColor(
				Double.valueOf(color.red * 255).intValue(),
				Double.valueOf(color.green * 255).intValue(),
				Double.valueOf(color.blue * 255).intValue());
	}

	public static final RGBColor LIME = new RGBColor(102, 255, 0);
	public static final RGBColor NONE = new RGBColor(0, 0, 0);
	public static final RGBColor RED = new RGBColor(255, 0, 0);
	public static final RGBColor ORANGE = new RGBColor(255, 185, 0);
	public static final RGBColor YELLOW = new RGBColor(255, 255, 0);
	public static final RGBColor GREEN = new RGBColor(0, 255, 0);
	public static final RGBColor BLUE = new RGBColor(0, 0, 255);
	public static final RGBColor AQUA = new RGBColor(0, 255, 255);
	public static final RGBColor PURPLE = new RGBColor(255, 0, 255);

	public String toString() {
		return "R: " + r + ", G:" + g + ", B:" + b;
	}

	public RGBWColor toRGBWCOlor() {
		return new RGBWColor(r, g, b, 0);
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof RGBColor) {
			RGBColor other = (RGBColor) o;
			return other == this || (r == other.r && b == other.b && g == other.g);
		}
		return false;
	}
}
