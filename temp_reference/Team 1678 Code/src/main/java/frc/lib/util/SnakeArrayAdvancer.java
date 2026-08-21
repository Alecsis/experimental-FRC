package frc.lib.util;

import java.util.Optional;

/**
 * @apiNote used for advancing arrays in an snake or linked list fashion
 */
public class SnakeArrayAdvancer<T> {

	private T buffer[];
	private int currentIndex;

	/**
	 *
	 * @param buffer array buffer
	 */
	public SnakeArrayAdvancer(T... buffer) {
		this.buffer = buffer;
		currentIndex = 0;
	}

	public boolean isEmpty() {
		return buffer.length == 0;
	}

	/**
	 * @apiNote sets {@link  #currentIndex} to 0
	 */
	public void zero() {
		currentIndex = 0;
	}

	public void max() {
		if (buffer.length > 0) {
			currentIndex = buffer.length - 1;
		}
		currentIndex = 0;
	}

	/**
	 *
	 * @param wantWrap if {@link #currentIndex} will return to 0 once {@link #currentIndex} is greater than the buffer size
	 * @return the advanced item or nothing
	 */
	public Optional<T> advance(boolean wantWrap) {
		if (buffer.length == 0) {
			return Optional.empty();
		}
		Optional<T> next = safeCurrent();
		if (++currentIndex == getBufferLength() || next.isEmpty()) {
			if (wantWrap) {
				zero();
			}
		}
		return next;
	}

	public Optional<T> deadvance(boolean wantWrap) {
		if (buffer.length == 0) {
			return Optional.empty();
		}
		Optional<T> current = safeCurrent();
		if (--currentIndex == -1 || current.isEmpty()) {
			if (wantWrap) {
				max();
			}
		}
		return current;
	}

	public int getCurrentIndex() {
		return currentIndex;
	}

	/**
	 *
	 * @return {@link currentIndex} index of {@link #buffer}
	 */
	public T current() {
		return buffer[currentIndex];
	}

	/**
	 * @apiNote returns the {@link #current()} in an safe manner (handles for errors by returning {@link Optional})
	 * @return an {@link Optional}
	 */
	public Optional<T> safeCurrent() {
		try {
			if (currentIndex >= buffer.length) {
				return Optional.empty();
			}
			if (buffer.length == 0) {
				return Optional.empty();
			}
			if (currentIndex < 0) {
				return Optional.empty();
			}
			T current = current();
			return Optional.of(current);
		} catch (Exception e) {
			return Optional.empty();
		}
	}

	public int getBufferLength() {
		return buffer.length;
	}
}
