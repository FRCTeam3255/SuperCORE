package com.frcteam3255.utils;

import java.io.*;
import java.util.Scanner;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.Filesystem;

/*SN motion profile contains the functions necessary to take a CSV file put in deploy and load it into
TalonSRX motion profiling*/
public class SN_MotionProfile {
	public BufferedTrajectoryPointStream pointsLeft;
	public BufferedTrajectoryPointStream pointsRight;

	String leftFilename;
	String rightFilename;

	public SN_MotionProfile(String a_leftName, String a_rightName) {
		pointsLeft = new BufferedTrajectoryPointStream();
		pointsRight = new BufferedTrajectoryPointStream();
		leftFilename = a_leftName;
		rightFilename = a_rightName;
		reload();

	}

	public void reload() {

		try {
			initBuffer(pointsLeft, reader(leftFilename), SN_MotionProfile.count(leftFilename));
		} catch (IOException e) {
			System.out.println("initBuffer failed :(. Is your file in deploy?");
			e.printStackTrace();
		}
		try {
			initBuffer(pointsRight, reader(rightFilename), SN_MotionProfile.count(rightFilename));
		} catch (IOException e) {
			System.out.println("initBuffer failed :(. Is your file in deploy?");
			e.printStackTrace();
		}

	}

	static double sensorUnitsPerTick = 600;

	public static void setSensorUnitsPerTick(double a_sensorUnitsPerTick) {
		sensorUnitsPerTick = a_sensorUnitsPerTick;
	}

	// counter taken from stackoverflow:
	// https://stackoverflow.com/questions/18009416/how-to-count-total-rows-in-csv-using-java
	public static int count(final String filename) throws IOException {
		final InputStream is = new BufferedInputStream(
				new FileInputStream(Filesystem.getDeployDirectory() + "/paths/" + filename));
		try {
			final byte[] c = new byte[1024];
			int count = 0;
			int readChars = 0;
			boolean empty = true;
			while ((readChars = is.read(c)) != -1) {
				empty = false;
				for (int i = 0; i < readChars; ++i) {
					if (c[i] == '\n') {
						++count;
					}
				}
			}
			return (count == 0 && !empty) ? 1 : count;
		} finally {
			is.close();
		}
	}

	public static void initBuffer(final BufferedTrajectoryPointStream bufferedStream, final double[][] profile,
			final int totalCnt) {

		final boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
		// since you can use negative numbers in profile).

		final TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
		// automatically, you can alloc just one

		/* clear the buffer, in case it was used elsewhere */
		bufferedStream.Clear();

		/* Insert every point into buffer, no limit on size */
		for (int i = 0; i < totalCnt; ++i) {

			final double direction = forward ? +1 : -1;
			final double positionRot = profile[i][0];
			final double velocityRPM = profile[i][1];
			final int durationMilliseconds = (int) profile[i][2];

			/* for each point, fill our structure and pass it to API */
			point.timeDur = durationMilliseconds;

			/* drive part */
			point.position = direction * positionRot * sensorUnitsPerTick; // Rotations
																			// =>
																			// sensor
																			// units
			point.velocity = direction * velocityRPM * sensorUnitsPerTick / 600.0; // RPM
																					// =>
																					// units
																					// per
																					// 100ms
			point.arbFeedFwd = 0; // good place for kS, kV, kA, etc...

			point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			point.zeroPos = false; /* don't reset sensor, this is done elsewhere since we have multiple sensors */
			point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
			point.useAuxPID = false; /* tell MPB that we aren't using both pids */

			bufferedStream.Write(point);
		}
	}

	public static double[][] reader(final String fileName) throws IOException {
		// Get scanner instance
		final File toScan = new File(Filesystem.getDeployDirectory() + "/paths", fileName);
		// System.out.println(Filesystem.getDeployDirectory()+"/paths");
		final Scanner scanner = new Scanner(toScan);
		double[][] output = new double[count(fileName) + 1][3];

		int i = 0;
		while (scanner.hasNextLine()) {
			final String line = scanner.nextLine();
			if (!line.contains("Position")) {
				int j = 0;
				final String[] fields = line.split(",");
				for (final String field : fields) {
					if (j < 3) {
						if (fileName.contains("neg") && (j == 0 || j == 1)) {

							output[i][j] = Double.parseDouble(field.replaceAll("\\s+", ""));
						} else {
							output[i][j] = -Double.parseDouble(field.replaceAll("\\s+", ""));
						}
					}
					j++;
				}
				i++;
			} else {

				output = new double[count(fileName) - 1][3];
			}
		}
		scanner.close();
		return output;

	}

}
