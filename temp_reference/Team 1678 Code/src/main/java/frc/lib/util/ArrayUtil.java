package frc.lib.util;

import edu.wpi.first.util.ErrorMessages;
import java.nio.Buffer;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class ArrayUtil {

	public static <T> T[] invertSource(T buffer[]) {
		if (buffer == null || buffer.length == 0) {
			return buffer;
		}

		int realLength = buffer.length - 1;

		for (int i = 0; i < buffer.length; i++) {
			int lowIndex = i;
			int highIndex = realLength - i;

			T low = buffer[lowIndex];
			T high = buffer[highIndex];

			buffer[lowIndex] = high;
			buffer[highIndex] = low;
		}

		return buffer;
	}

	public static <T> int runFunctionAndSkipNull(T buffer[], BiConsumer<Integer, T> function) {
		if (buffer == null) {
			return 0;
		}
		int iterations = 0;
		for (int i = 0; i < buffer.length; i++) {
			T element = buffer[i];
			try {
				if (element != null) {
					function.accept(i, element);
					iterations++;
				}
			} catch (Exception e) {

			}
		}
		return iterations;
	}

	public static <T> int foreachWithoutNull(T buffer[], Consumer<T> function) {
		if (buffer == null) return 0;
		int iterations = 0;
		for (T element : buffer) {
			try {
				if (element != null) {
					function.accept(element);
					iterations++;
				}
			} catch (Exception e) {
			}
		}
		return iterations;
	}

	@SuppressWarnings("hiding")
	public static <T, Buffer extends List<T>> int appendArrayToList(T source[], Buffer destination) {
		int iterations = 0;
		try {
			ErrorMessages.requireNonNullParam(source, "source", "cpyArrayToList");
			ErrorMessages.requireNonNullParam(destination, "destination", "cpyArrayToList");

			for (T element : source) {
				destination.add(element);
				iterations++;
			}
		} finally {
		}
		return iterations;
	}

	@SuppressWarnings("hiding")
	public static <T, Buffer extends List<T>> int cpyListToArray(Buffer source, T destination[]) {
		if (source.size() == 0) {
			return 0;
		}

		if (destination.length < source.size()) {
			return -1;
		}

		int iterations = 0;

		for (int i = 0; i < source.size(); i++) {
			destination[i] = source.get(i);
		}
		return iterations;
	}
}
