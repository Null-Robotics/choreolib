// Copyright (c) Choreo contributors

package dev.nullrobotics;

import dev.nullrobotics.sample.SwerveSample;
import dev.nullrobotics.trajectory.EventMarker;
import dev.nullrobotics.trajectory.ProjectFile;
import dev.nullrobotics.trajectory.Trajectory;
import dev.nullrobotics.trajectory.TrajectorySample;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.google.gson.JsonSyntaxException;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

/** Utilities to load and follow Choreo Trajectories */
public final class Choreo {
  private static final Gson GSON =
      new GsonBuilder()
          .registerTypeAdapter(EventMarker.class, new EventMarker.Deserializer())
          .create();
  private static final String TRAJECTORY_FILE_EXTENSION = ".traj";
  private static final String SPEC_VERSION = "v2025.0.0";

  private static File CHOREO_DIR = new File(AppUtil.FIRST_FOLDER, "/choreo/");

  private static Optional<ProjectFile> LAZY_PROJECT_FILE = Optional.empty();

  public static final Logger LOGGER = LoggerFactory.getLogger("Choreo");

  /** This should only be used for unit testing. */
  static void setChoreoDir(final File choreoDir) {
    CHOREO_DIR = choreoDir;
  }

  /**
   * Gets the project file from the deploy directory. Choreolib expects a .chor file to be placed in
   * src/main/deploy/choreo.
   *
   * <p>The result is cached after the first call.
   *
   * @return the project file
   */
  public static ProjectFile getProjectFile() {
    if (LAZY_PROJECT_FILE.isPresent()) {
      return LAZY_PROJECT_FILE.get();
    }
    try {
      // find the first file that ends with a .chor extension
      final File[] projectFiles = CHOREO_DIR.listFiles((dir, name) -> name.endsWith(".chor"));
      if (projectFiles.length == 0) {
        throw new RuntimeException("Could not find project file in deploy directory");
      } else if (projectFiles.length > 1) {
        throw new RuntimeException("Found multiple project files in deploy directory");
      }
      final BufferedReader reader = new BufferedReader(new FileReader(projectFiles[0]));
      final String str = reader.lines().reduce("", (a, b) -> a + b);
      reader.close();
      final JsonObject json = GSON.fromJson(str, JsonObject.class);
      final String version = json.get("version").getAsString();
      if (!SPEC_VERSION.equals(version)) {
        throw new RuntimeException(
            ".chor project file: Wrong version " + version + ". Expected " + SPEC_VERSION);
      }
      LAZY_PROJECT_FILE = Optional.of(GSON.fromJson(str, ProjectFile.class));
    } catch (final JsonSyntaxException ex) {
      throw new RuntimeException("Could not parse project file", ex);
    } catch (final IOException ex) {
      throw new RuntimeException("Could not find project file", ex);
    }
    return LAZY_PROJECT_FILE.get();
  }

  /**
   * This interface exists as a type alias. A TrajectoryLogger has a signature of ({@link
   * Trajectory}, {@link Boolean})-&gt;void, where the function consumes a trajectory and a boolean
   * indicating whether the trajectory is starting or finishing.
   *
   * @param <SampleType> DifferentialSample or SwerveSample.
   */
  public interface TrajectoryLogger<SampleType extends TrajectorySample<SampleType>>
      extends BiConsumer<Trajectory<SampleType>, Boolean> {}

  /** Default constructor. */
  private Choreo() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Load a trajectory from the deploy directory. Choreolib expects .traj files to be placed in
   * src/main/deploy/choreo/[trajectoryName].traj.
   *
   * @param <SampleType> The type of samples in the trajectory.
   * @param trajectoryName The path name in Choreo, which matches the file name in the deploy
   *     directory, file extension is optional.
   * @return The loaded trajectory, or `Optional.empty()` if the trajectory could not be loaded.
   */
  @SuppressWarnings("unchecked")
  public static <SampleType extends TrajectorySample<SampleType>> Optional<Trajectory<SampleType>> loadTrajectory(String trajectoryName) {

    if (trajectoryName.endsWith(TRAJECTORY_FILE_EXTENSION)) {
      trajectoryName =
          trajectoryName.substring(0, trajectoryName.length() - TRAJECTORY_FILE_EXTENSION.length());
    }
    final File trajectoryFile = new File(CHOREO_DIR, trajectoryName + TRAJECTORY_FILE_EXTENSION);

    Choreo.LOGGER.info("Loading trajectory {}",   trajectoryFile.getAbsolutePath());
    try {
      final BufferedReader reader = new BufferedReader(new FileReader(trajectoryFile));
      final String str = reader.lines().reduce("", (a, b) -> a + b);
      reader.close();
      final Trajectory<SampleType> trajectory =
          (Trajectory<SampleType>) loadTrajectoryString(str, getProjectFile());
      return Optional.of(trajectory);
    } catch (final FileNotFoundException ex) {
      Choreo.LOGGER.error("Could not find trajectory file: " + trajectoryFile, ex);
    } catch (final JsonSyntaxException ex) {
      Choreo.LOGGER.error("Could not parse trajectory file: " + trajectoryFile, ex);
    } catch (final Exception ex) {
      Choreo.LOGGER.error(ex.getMessage(), ex.getStackTrace());
    }
    return Optional.empty();
  }

  /**
   * Load a trajectory from a string.
   *
   * @param trajectoryJsonString The JSON string.
   * @param projectFile The project file.
   * @return The loaded trajectory, or `empty std::optional` if the trajectory could not be loaded.
   */
  static Trajectory<? extends TrajectorySample<?>> loadTrajectoryString(
          final String trajectoryJsonString, final ProjectFile projectFile) {
    final JsonObject wholeTrajectory = GSON.fromJson(trajectoryJsonString, JsonObject.class);

    final String name = wholeTrajectory.get("name").getAsString();
    final String version = wholeTrajectory.get("version").getAsString();
    if (!SPEC_VERSION.equals(version)) {
      throw new RuntimeException(
          name + ".traj: Wrong version: " + version + ". Expected " + SPEC_VERSION);
    }
    // Filter out markers with negative timestamps or empty names
    final List<EventMarker> unfilteredEvents =
        new ArrayList<EventMarker>(
            Arrays.asList(GSON.fromJson(wholeTrajectory.get("events"), EventMarker[].class)));
    unfilteredEvents.removeIf(marker -> marker.timestamp < 0 || marker.event.length() == 0);
    final EventMarker[] events = new EventMarker[unfilteredEvents.size()];
    unfilteredEvents.toArray(events);

    final JsonObject trajectoryObj = wholeTrajectory.getAsJsonObject("trajectory");
    Integer[] splits = GSON.fromJson(trajectoryObj.get("splits"), Integer[].class);
    if (splits.length == 0 || splits[0] != 0) {
      final Integer[] newArray = new Integer[splits.length + 1];
      newArray[0] = 0;
      System.arraycopy(splits, 0, newArray, 1, splits.length);
      splits = newArray;
    }
    if (projectFile.type.equals("Swerve")) {
      final SwerveSample[] samples = GSON.fromJson(trajectoryObj.get("samples"), SwerveSample[].class);
      return new Trajectory<SwerveSample>(name, List.of(samples), List.of(splits), List.of(events));
    } else if (projectFile.type.equals("Differential")) {
      throw new RuntimeException("Differential samples are not supported");
    } else {
      throw new RuntimeException("Unknown project type: " + projectFile.type);
    }
  }

  /**
   * A utility for caching loaded trajectories. This allows for loading trajectories only once, and
   * then reusing them.
   */
  public static class TrajectoryCache {
    private final Map<String, Trajectory<?>> cache;

    /** Creates a new TrajectoryCache with a normal {@link HashMap} as the cache. */
    public TrajectoryCache() {
        this.cache = new HashMap<>();
    }

    /**
     * Creates a new TrajectoryCache with a custom cache.
     *
     * <p>this could be useful if you want to use a concurrent map or a map with a maximum size.
     *
     * @param cache The cache to use.
     */
    public TrajectoryCache(final Map<String, Trajectory<?>> cache) {
      this.cache = cache;
    }

    /**
     * Load a trajectory from the deploy directory. Choreolib expects .traj files to be placed in
     * src/main/deploy/choreo/[trajectoryName].traj.
     *
     * <p>This method will cache the loaded trajectory and reused it if it is requested again.
     *
     * @param trajectoryName the path name in Choreo, which matches the file name in the deploy
     *     directory, file extension is optional.
     * @return the loaded trajectory, or `Optional.empty()` if the trajectory could not be loaded.
     * @see Choreo#loadTrajectory(String)
     */
    public Optional<? extends Trajectory<?>> loadTrajectory(final String trajectoryName) {
      if (this.cache.containsKey(trajectoryName)) {
        return Optional.of(this.cache.get(trajectoryName));
      } else {
        return Choreo.loadTrajectory(trajectoryName)
            .map(
                trajectory -> {
                    this.cache.put(trajectoryName, trajectory);
                  return trajectory;
                });
      }
    }

    /**
     * Load a section of a split trajectory from the deploy directory. Choreolib expects .traj files
     * to be placed in src/main/deploy/choreo/[trajectoryName].traj.
     *
     * <p>This method will cache the loaded trajectory and reused it if it is requested again. The
     * trajectory that is split off of will also be cached.
     *
     * @param trajectoryName the path name in Choreo, which matches the file name in the deploy
     *     directory, file extension is optional.
     * @param splitIndex the index of the split trajectory to load
     * @return the loaded trajectory, or `Optional.empty()` if the trajectory could not be loaded.
     * @see Choreo#loadTrajectory(String)
     */
    public Optional<? extends Trajectory<?>> loadTrajectory(final String trajectoryName, final int splitIndex) {
      // make the key something that could never possibly be a valid trajectory name
      final String key = trajectoryName + ".:." + splitIndex;
      if (this.cache.containsKey(key)) {
        return Optional.of(this.cache.get(key));
      } else if (this.cache.containsKey(trajectoryName)) {
        return this.cache
            .get(trajectoryName)
            .getSplit(splitIndex)
            .map(
                trajectory -> {
                    this.cache.put(key, trajectory);
                  return trajectory;
                });
      } else {
        return Choreo.loadTrajectory(trajectoryName)
            .flatMap(
                trajectory -> {
                    this.cache.put(trajectoryName, trajectory);
                  return trajectory
                      .getSplit(splitIndex)
                      .map(
                          split -> {
                              this.cache.put(key, split);
                            return split;
                          });
                });
      }
    }

    /** Clear the cache. */
    public void clear() {
        this.cache.clear();
    }
  }

}
