package frc.robot.sim.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class ScenarioJsonTest {
  private static final Path SIM_DIR = Path.of("simulation");

  private static List<Path> jsonFiles(String sub) throws IOException {
    try (Stream<Path> s = Files.list(SIM_DIR.resolve(sub))) {
      return s.filter(p -> p.toString().endsWith(".json")).sorted().toList();
    }
  }

  @Test
  void everyScenarioFileParsesValidatesAndRoundTripsExactly() throws IOException {
    FieldLocations locations = ScenarioJson.readFieldLocations(SIM_DIR.resolve("field-locations.json"));
    List<Path> files = jsonFiles("scenarios");
    assertFalse(files.isEmpty());
    for (Path file : files) {
      Scenario s = ScenarioJson.readScenario(file);
      assertEquals(List.of(), ScenarioJson.validate(s, locations, null, null), file.toString());
      String original = Files.readString(file, StandardCharsets.UTF_8);
      assertEquals(original, ScenarioJson.toJson(ScenarioJson.scenarioToMap(s)), "round trip of " + file);
    }
  }

  @Test
  void suitesReferenceExistingScenarios() throws IOException {
    for (Path file : jsonFiles("suites")) {
      Suite suite = ScenarioJson.readSuite(file);
      assertFalse(suite.runs().isEmpty(), file.toString());
      for (Suite.Run run : suite.runs()) {
        if (!run.allAutos()) {
          assertTrue(Files.exists(SIM_DIR.resolve("scenarios").resolve(run.scenario() + ".json")),
              file + " references missing scenario " + run.scenario());
        }
      }
      assertEquals(Files.readString(file, StandardCharsets.UTF_8), ScenarioJson.toJson(ScenarioJson.suiteToMap(suite)));
    }
  }

  @Test
  void fieldLocationsRoundTrip() throws IOException {
    Path file = SIM_DIR.resolve("field-locations.json");
    FieldLocations f = ScenarioJson.readFieldLocations(file);
    assertTrue(f.locations().containsKey("AllianceZoneTestSpot"));
    assertEquals(Files.readString(file, StandardCharsets.UTF_8), ScenarioJson.toJson(ScenarioJson.fieldLocationsToMap(f)));
  }

  @Test
  void eventTimesMatchOldJavaArithmetic() throws IOException {
    // Old SUBSYSTEM_TEST: t=0 pose, +4 intake, +1 stop, +2, +1, +0 turn, ... final END at 19.5
    Scenario s = ScenarioJson.readScenario(SIM_DIR.resolve("scenarios/subsystem-test.json"));
    double[] times = s.getTeleop().absoluteTimes();
    assertEquals(0.0, times[0], 1e-9);
    assertEquals(4.0, times[1], 1e-9);
    assertEquals(8.0, times[5], 1e-9); // "Turn 1" shares a time with "Drive 2"
    assertEquals(19.5, s.getTeleop().endTime(), 1e-9);
  }

  @Test
  void allianceFlipMirrorsThroughFieldCentre() {
    FieldLocations f = new FieldLocations();
    f.locations().put("spot", new Pose(2, 1, 0));
    Pose red = f.resolve("spot", null, ScenarioEvent.Frame.ALLIANCE, SimAlliance.RED);
    assertEquals(f.getFieldLength() - 2, red.x(), 1e-9);
    assertEquals(f.getFieldWidth() - 1, red.y(), 1e-9);
    assertEquals(180.0, red.deg(), 1e-9);
    Pose abs = f.resolve("spot", null, ScenarioEvent.Frame.ABSOLUTE, SimAlliance.RED);
    assertEquals(new Pose(2, 1, 0), abs);
  }

  @Test
  void parseErrorsNameTheJsonPath() {
    String bad = "{ \"name\": \"x\", \"teleop\": { \"events\": [ { \"wait\": 1, \"type\": \"button\", \"button\": \"Z\", \"action\": \"hold\" } ] } }";
    ScenarioJson.FormatException e =
        assertThrows(ScenarioJson.FormatException.class, () -> ScenarioJson.parseScenario(ScenarioJson.parse(bad)));
    assertTrue(e.getMessage().startsWith("teleop.events[0].button:"), e.getMessage());

    String badType = "{ \"name\": \"x\", \"teleop\": { \"events\": [ { \"type\": \"jump\" } ] } }";
    e = assertThrows(ScenarioJson.FormatException.class, () -> ScenarioJson.parseScenario(ScenarioJson.parse(badType)));
    assertTrue(e.getMessage().contains("unknown event type"), e.getMessage());
  }

  @Test
  void validationCatchesSemanticProblems() throws IOException {
    String json = """
        { "name": "bad", "gameData": "Q",
          "teleop": { "events": [
            { "wait": 0, "type": "setPose", "location": "Nowhere" },
            { "wait": 0, "type": "drive", "x": 0.9, "y": 0.9 },
            { "wait": 0, "type": "end" },
            { "wait": -1, "type": "expect", "probe": "intake.state" }
          ] } }
        """;
    Scenario s = ScenarioJson.parseScenario(ScenarioJson.parse(json));
    List<String> problems = ScenarioJson.validate(s, new FieldLocations(), null, List.of("pose"));
    String all = String.join("\n", problems);
    assertTrue(all.contains("gameData"), all);
    assertTrue(all.contains("unknown location \"Nowhere\""), all);
    assertTrue(all.contains("vector length"), all);
    assertTrue(all.contains("must be the last event"), all);
    assertTrue(all.contains("wait must be"), all);
    assertTrue(all.contains("unknown probe"), all);
    assertTrue(all.contains("needs \"equals\""), all);
  }

  @Test
  void expectEventRoundTrips() throws IOException {
    String json = """
        { "name": "e", "teleop": { "events": [
          { "wait": 1, "type": "expect", "probe": "pose", "x": 1, "y": 2, "deg": 90, "frame": "absolute", "tolerance": 0.3, "within": 2 },
          { "wait": 0, "type": "expect", "probe": "shooter.hoodAtGoal", "equals": true }
        ] } }
        """;
    Scenario s = ScenarioJson.parseScenario(ScenarioJson.parse(json));
    ScenarioEvent.Expect pose = assertInstanceOf(ScenarioEvent.Expect.class, s.getTeleop().events().get(0));
    assertEquals(new Pose(1, 2, 90), pose.pose());
    assertEquals(ScenarioEvent.Frame.ABSOLUTE, pose.frame());
    ScenarioEvent.Expect eq = assertInstanceOf(ScenarioEvent.Expect.class, s.getTeleop().events().get(1));
    assertEquals("true", eq.equalsValue());
    Scenario again = ScenarioJson.parseScenario(ScenarioJson.parse(ScenarioJson.toJson(ScenarioJson.scenarioToMap(s))));
    assertEquals(s.getTeleop().events(), again.getTeleop().events());
  }

  @Test
  void runConfigAndReportRoundTrip() throws IOException {
    RunConfig c = new RunConfig();
    c.scenarios().add("trenches");
    c.alliances().add(SimAlliance.RED);
    RunConfig back = ScenarioJson.parseRunConfig(ScenarioJson.parse(ScenarioJson.toJson(ScenarioJson.runConfigToMap(c))));
    assertEquals(List.of("trenches"), back.scenarios());
    assertEquals(List.of(SimAlliance.RED), back.alliances());

    RunReport r = new RunReport();
    RunReport.Test t = new RunReport.Test("Trenches", "trenches", SimAlliance.BLUE);
    t.checks().add(new RunReport.Check("teleop", 3, "at trench", false, "(1.00, 2.00, 0°)", 5.5));
    r.tests().add(t);
    RunReport rb = ScenarioJson.parseRunReport(ScenarioJson.parse(ScenarioJson.toJson(ScenarioJson.runReportToMap(r))));
    assertFalse(rb.passed());
    assertEquals(t.checks(), rb.tests().get(0).checks());
  }
}
