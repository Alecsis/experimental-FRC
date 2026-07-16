package frc.lib.util;

import edu.wpi.first.util.ErrorMessages;
import java.util.ArrayList;

public class LoopableHandler implements Loopable {

	private final ArrayList<Loopable> loops = new ArrayList<>();
	private final ArrayList<Loopable> blacklistedLoops = new ArrayList<>();

	public LoopableHandler(boolean wantBlacklistSelf) {
		if (wantBlacklistSelf) {
			blacklist(this);
		}
	}

	public LoopableHandler() {
		this(true);
	}

	public void blacklist(Loopable loopable) {
		blacklistedLoops.add(loopable);
	}

	public void unblacklist(Loopable loopable) {
		blacklistedLoops.remove(loopable);
	}

	public void unblacklistSelf() {
		unblacklist(this);
	}

	public boolean isBlacklisted(Loopable loopable) {
		if (blacklistedLoops.isEmpty()) {
			return false;
		}
		if (blacklistedLoops.size() == 1) {
			return blacklistedLoops.get(0).equals(loopable);
		} else {
			return blacklistedLoops.contains(loopable);
		}
	}

	public boolean addLoop(Loopable loopable) {
		ErrorMessages.requireNonNullParam(loopable, "loopable", "addLoop");
		if (isBlacklisted(loopable)) {
			return false;
		} else {
			loops.add(loopable);
			return true;
		}
	}

	@Override
	public void loop() {
		for (Loopable loopable : loops) {
			loopable.loop();
		}
	}
}
