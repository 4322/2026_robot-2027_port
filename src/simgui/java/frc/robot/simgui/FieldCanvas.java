package frc.robot.simgui;

import frc.robot.sim.scenario.FieldLocations;
import frc.robot.sim.scenario.Pose;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import javafx.geometry.Point2D;
import javafx.geometry.VPos;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Region;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontPosture;
import javafx.scene.text.FontWeight;
import javafx.scene.text.Text;
import javafx.scene.text.TextAlignment;

/**
 * Top-down field view (BLUE driver station on the left, +y up, like AdvantageScope's 2D field).
 * Draws field shapes from {@link FieldLocations} plus the markers, arrows and path lines the owning
 * controller supplies, and turns mouse drags into edit callbacks. Resizes with its parent.
 */
final class FieldCanvas extends Region {
  /** Approximate robot footprint for drawing, metres. */
  private static final double ROBOT_SIZE = 0.85;
  /** Distance of the rotation handle from the robot centre, metres. */
  private static final double HANDLE_DISTANCE = 0.8;
  /** Arrow length for a full (1.0) stick deflection, metres. */
  static final double STICK_ARROW_SCALE = 1.5;
  private static final double MARGIN_M = 0.6;

  /**
   * A robot-shaped marker. {@code key} identifies it in callbacks. Outline markers (expects, the
   * end-of-segment estimate) are drawn hollow and dashed so they don't hide real poses.
   */
  record Marker(Object key, Pose pose, String label, Color color, boolean selected, boolean draggable, boolean outline) {}

  /** A stick-vector arrow from (x0, y0) to (x1, y1). Its tip is draggable when {@code draggable}. */
  record Arrow(Object key, double x0, double y0, double x1, double y1, Color color, boolean selected, boolean draggable) {}

  /** Estimated robot path segment (drawn dashed). */
  record PathLine(double x0, double y0, double x1, double y1) {}

  /** What to draw, recomputed on every redraw. */
  record Scene(List<Marker> markers, List<Arrow> arrows, List<PathLine> path, String caption) {
    static Scene empty() {
      return new Scene(List.of(), List.of(), List.of(), null);
    }
  }

  /** Edit callbacks. {@code finished} is true on mouse release. */
  interface Listener {
    default void selected(Object key) {}

    default void markerMoved(Object key, double x, double y, boolean finished) {}

    default void markerRotated(Object key, double deg, boolean finished) {}

    default void arrowTipMoved(Object key, double x, double y, boolean finished) {}
  }

  private enum DragKind {
    MOVE,
    ROTATE,
    ARROW_TIP
  }

  private static final Color BACKGROUND = Color.web("#1b1e22");
  private static final Color CARPET = Color.web("#33423a");
  private static final Color FIELD_LINE = Color.web("#d8dde2");

  private final Canvas canvas = new Canvas();
  private final Supplier<FieldLocations> field;
  private Supplier<Scene> scene = Scene::empty;
  private Listener listener = new Listener() {};
  private Object dragKey;
  private DragKind dragKind;
  private Point2D dragOffset;
  private String hover = "";

  FieldCanvas(Supplier<FieldLocations> field) {
    this.field = field;
    getChildren().add(canvas);
    setMinSize(200, 120);
    setPrefSize(820, 440);
    canvas.setOnMousePressed(this::pressed);
    canvas.setOnMouseDragged(e -> drag(e, false));
    canvas.setOnMouseReleased(e -> {
      if (dragKey != null) {
        drag(e, true);
      }
      dragKey = null;
      dragKind = null;
    });
    canvas.setOnMouseMoved(e -> {
      hover = hoverText(new Point2D(e.getX(), e.getY()));
      redraw();
    });
  }

  @Override
  protected void layoutChildren() {
    canvas.setWidth(getWidth());
    canvas.setHeight(getHeight());
    redraw();
  }

  void setScene(Supplier<Scene> scene) {
    this.scene = scene;
    redraw();
  }

  void setListener(Listener listener) {
    this.listener = listener;
  }

  // ------------------------------------------------------------------ coordinates

  private double scale() {
    FieldLocations f = field.get();
    double sx = (getWidth() - 2.0) / (f.getFieldLength() + 2 * MARGIN_M);
    double sy = (getHeight() - 2.0) / (f.getFieldWidth() + 2 * MARGIN_M);
    return Math.max(1e-3, Math.min(sx, sy));
  }

  /** Field metres → canvas pixels. */
  private Point2D toScreen(double x, double y) {
    FieldLocations f = field.get();
    double s = scale();
    double ox = (getWidth() - f.getFieldLength() * s) / 2.0;
    double oy = (getHeight() + f.getFieldWidth() * s) / 2.0;
    return new Point2D(ox + x * s, oy - y * s);
  }

  /** Canvas pixels → field metres. */
  private Point2D toField(Point2D p) {
    FieldLocations f = field.get();
    double s = scale();
    double ox = (getWidth() - f.getFieldLength() * s) / 2.0;
    double oy = (getHeight() + f.getFieldWidth() * s) / 2.0;
    return new Point2D((p.getX() - ox) / s, (oy - p.getY()) / s);
  }

  // ------------------------------------------------------------------ mouse

  private void pressed(MouseEvent e) {
    Point2D fp = toField(new Point2D(e.getX(), e.getY()));
    Scene sc = scene.get();
    double tol = 10.0 / scale(); // 10 px in metres
    for (Marker m : sc.markers()) { // 1. rotation handle of the selected marker
      if (m.selected() && m.draggable() && fp.distance(handlePoint(m.pose())) <= tol) {
        begin(m.key(), DragKind.ROTATE, Point2D.ZERO);
        return;
      }
    }
    for (Arrow a : sc.arrows()) { // 2. arrow tips
      if (a.draggable() && fp.distance(a.x1(), a.y1()) <= tol) {
        listener.selected(a.key());
        begin(a.key(), DragKind.ARROW_TIP, Point2D.ZERO);
        return;
      }
    }
    for (int i = sc.markers().size() - 1; i >= 0; i--) { // 3. marker bodies, topmost first
      Marker m = sc.markers().get(i);
      if (fp.distance(m.pose().x(), m.pose().y()) <= ROBOT_SIZE * 0.6) {
        listener.selected(m.key());
        if (m.draggable()) {
          begin(m.key(), DragKind.MOVE, new Point2D(m.pose().x() - fp.getX(), m.pose().y() - fp.getY()));
        }
        return;
      }
    }
    for (Arrow a : sc.arrows()) { // 4. arrow shafts
      if (segmentDistance(fp, a) <= tol) {
        listener.selected(a.key());
        return;
      }
    }
  }

  private void begin(Object key, DragKind kind, Point2D offset) {
    dragKey = key;
    dragKind = kind;
    dragOffset = offset;
  }

  private void drag(MouseEvent e, boolean finished) {
    if (dragKey == null) {
      return;
    }
    Point2D fp = toField(new Point2D(e.getX(), e.getY()));
    switch (dragKind) {
      case MOVE -> listener.markerMoved(dragKey, round(fp.getX() + dragOffset.getX()), round(fp.getY() + dragOffset.getY()), finished);
      case ROTATE -> {
        Marker m = findMarker(dragKey);
        if (m != null) {
          double deg = Math.toDegrees(Math.atan2(fp.getY() - m.pose().y(), fp.getX() - m.pose().x()));
          listener.markerRotated(dragKey, Math.round(deg / 5.0) * 5.0, finished); // 5° steps
        }
      }
      case ARROW_TIP -> listener.arrowTipMoved(dragKey, fp.getX(), fp.getY(), finished);
    }
    redraw();
  }

  /** Rounds to centimetres so files don't fill with noise digits. */
  private static double round(double v) {
    return Math.round(v * 100.0) / 100.0;
  }

  private Marker findMarker(Object key) {
    for (Marker m : scene.get().markers()) {
      if (m.key().equals(key)) {
        return m;
      }
    }
    return null;
  }

  private static Point2D handlePoint(Pose p) {
    double r = Math.toRadians(p.deg());
    return new Point2D(p.x() + HANDLE_DISTANCE * Math.cos(r), p.y() + HANDLE_DISTANCE * Math.sin(r));
  }

  private static double segmentDistance(Point2D p, Arrow a) {
    double dx = a.x1() - a.x0();
    double dy = a.y1() - a.y0();
    double len2 = dx * dx + dy * dy;
    double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((p.getX() - a.x0()) * dx + (p.getY() - a.y0()) * dy) / len2));
    return p.distance(a.x0() + t * dx, a.y0() + t * dy);
  }

  private String hoverText(Point2D screen) {
    Point2D fp = toField(screen);
    for (Marker m : scene.get().markers()) {
      if (fp.distance(m.pose().x(), m.pose().y()) <= ROBOT_SIZE * 0.6) {
        return m.label() + "  " + m.pose() + (m.draggable() ? "  · drag to move, drag the dot to rotate" : "");
      }
    }
    for (FieldLocations.Feature f : field.get().features()) {
      if (fp.getX() >= Math.min(f.x1(), f.x2()) && fp.getX() <= Math.max(f.x1(), f.x2())
          && fp.getY() >= Math.min(f.y1(), f.y2()) && fp.getY() <= Math.max(f.y1(), f.y2())) {
        return f.name();
      }
    }
    return String.format("(%.2f, %.2f)", fp.getX(), fp.getY());
  }

  // ------------------------------------------------------------------ drawing

  void redraw() {
    double w = getWidth();
    double h = getHeight();
    if (w <= 0 || h <= 0) {
      return;
    }
    GraphicsContext g = canvas.getGraphicsContext2D();
    g.setLineDashes();
    g.setFill(BACKGROUND);
    g.fillRect(0, 0, w, h);
    FieldLocations f = field.get();
    double s = scale();

    // carpet and alliance zones
    Point2D tl = toScreen(0, f.getFieldWidth());
    double fw = f.getFieldLength() * s;
    double fh = f.getFieldWidth() * s;
    g.setFill(CARPET);
    g.fillRect(tl.getX(), tl.getY(), fw, fh);
    double zone = Math.min(4.6, f.getFieldLength() / 3.5) * s;
    g.setFill(Color.rgb(50, 110, 230, 0.16));
    g.fillRect(tl.getX(), tl.getY(), zone, fh);
    g.setFill(Color.rgb(230, 60, 60, 0.16));
    g.fillRect(tl.getX() + fw - zone, tl.getY(), zone, fh);
    g.setStroke(FIELD_LINE);
    g.setLineWidth(2);
    g.strokeRect(tl.getX(), tl.getY(), fw, fh);
    g.setLineWidth(1);
    g.strokeLine(tl.getX() + fw / 2, tl.getY(), tl.getX() + fw / 2, tl.getY() + fh);

    // features
    for (FieldLocations.Feature ft : f.features()) {
      Point2D a = toScreen(Math.min(ft.x1(), ft.x2()), Math.max(ft.y1(), ft.y2()));
      double rw = Math.abs(ft.x2() - ft.x1()) * s;
      double rh = Math.abs(ft.y2() - ft.y1()) * s;
      Color col = parseColor(ft.color());
      g.setFill(col.deriveColor(0, 1, 1, 0.55));
      g.fillRect(a.getX(), a.getY(), rw, rh);
      g.setStroke(col.brighter());
      g.strokeRect(a.getX(), a.getY(), rw, rh);
    }

    // driver station labels
    g.setFont(Font.font("System", FontWeight.BOLD, 11));
    g.setTextAlign(TextAlignment.CENTER);
    g.setTextBaseline(VPos.CENTER);
    drawRotated(g, "BLUE DS", tl.getX() - 10, tl.getY() + fh / 2, -90, Color.web("#7fa7ff"));
    drawRotated(g, "RED DS", tl.getX() + fw + 10, tl.getY() + fh / 2, 90, Color.web("#ff8a8a"));
    g.setTextAlign(TextAlignment.LEFT);
    g.setTextBaseline(VPos.BASELINE);

    Scene sc = scene.get();

    // estimated path
    g.setStroke(Color.rgb(255, 255, 255, 0.55));
    g.setLineWidth(1.5);
    g.setLineDashes(6, 5);
    for (PathLine pl : sc.path()) {
      Point2D p0 = toScreen(pl.x0(), pl.y0());
      Point2D p1 = toScreen(pl.x1(), pl.y1());
      g.strokeLine(p0.getX(), p0.getY(), p1.getX(), p1.getY());
    }
    g.setLineDashes();

    // markers; labels of markers at (nearly) the same spot are stacked instead of overprinted
    List<Point2D> labelled = new ArrayList<>();
    for (Marker m : sc.markers()) {
      int stack = 0;
      for (Point2D q : labelled) {
        if (q.distance(m.pose().x(), m.pose().y()) < 0.5) {
          stack++;
        }
      }
      labelled.add(new Point2D(m.pose().x(), m.pose().y()));
      drawRobot(g, m, stack);
    }

    for (Arrow a : sc.arrows()) {
      drawArrow(g, a);
    }

    // caption (bottom left) and hover info (bottom right)
    g.setFont(Font.font("System", 11));
    g.setFill(Color.web("#aab2bb"));
    if (sc.caption() != null) {
      g.fillText(sc.caption(), 8, h - 8);
    }
    g.setTextAlign(TextAlignment.RIGHT);
    g.fillText(hover, w - 8, 16);
    g.setTextAlign(TextAlignment.LEFT);
  }

  private void drawRobot(GraphicsContext g, Marker m, int labelLine) {
    double half = ROBOT_SIZE * scale() / 2;
    Point2D c = toScreen(m.pose().x(), m.pose().y());
    g.save();
    g.translate(c.getX(), c.getY());
    g.rotate(-m.pose().deg());
    if (m.outline()) {
      g.setStroke(m.selected() ? Color.WHITE : m.color());
      g.setLineWidth(m.selected() ? 3 : 2);
      g.setLineDashes(5, 4);
      g.strokeRect(-half, -half, 2 * half, 2 * half);
      g.setLineDashes();
    } else {
      g.setFill(m.color().deriveColor(0, 1, 1, m.selected() ? 0.95 : 0.7));
      g.fillRect(-half, -half, 2 * half, 2 * half);
      g.setStroke(m.selected() ? Color.WHITE : m.color().darker());
      g.setLineWidth(m.selected() ? 2.5 : 1.2);
      g.strokeRect(-half, -half, 2 * half, 2 * half);
      g.setFill(Color.WHITE); // front indicator
      g.fillPolygon(new double[] {half, half * 0.35, half * 0.35}, new double[] {0, -half * 0.45, half * 0.45}, 3);
    }
    g.restore();
    if (m.selected() && m.draggable()) {
      Point2D hp = handlePoint(m.pose());
      Point2D hs = toScreen(hp.getX(), hp.getY());
      g.setStroke(Color.WHITE);
      g.setLineWidth(1);
      g.strokeLine(c.getX(), c.getY(), hs.getX(), hs.getY());
      g.setFill(Color.WHITE);
      g.fillOval(hs.getX() - 5, hs.getY() - 5, 10, 10);
    }
    if (m.label() != null) {
      Font font = Font.font("System", m.outline() ? FontWeight.NORMAL : FontWeight.BOLD,
          m.outline() ? FontPosture.ITALIC : FontPosture.REGULAR, 11);
      Text measure = new Text(m.label());
      measure.setFont(font);
      double tw = measure.getLayoutBounds().getWidth();
      double th = measure.getLayoutBounds().getHeight();
      double x = c.getX() + half + 3;
      double y = c.getY() - half + labelLine * (th + 1);
      g.setFill(Color.rgb(0, 0, 0, 0.55));
      g.fillRect(x - 2, y - th + 2, tw + 4, th);
      g.setFont(font);
      g.setFill(m.outline() ? m.color().brighter() : Color.WHITE);
      g.fillText(m.label(), x, y);
    }
  }

  private void drawArrow(GraphicsContext g, Arrow a) {
    Point2D p0 = toScreen(a.x0(), a.y0());
    Point2D p1 = toScreen(a.x1(), a.y1());
    Color col = a.selected() ? Color.WHITE : a.color();
    g.setStroke(col);
    g.setFill(col);
    g.setLineWidth(a.selected() ? 3.5 : 2.2);
    g.strokeLine(p0.getX(), p0.getY(), p1.getX(), p1.getY());
    double ang = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
    double len = 11;
    g.fillPolygon(
        new double[] {p1.getX(), p1.getX() - len * Math.cos(ang - 0.4), p1.getX() - len * Math.cos(ang + 0.4)},
        new double[] {p1.getY(), p1.getY() - len * Math.sin(ang - 0.4), p1.getY() - len * Math.sin(ang + 0.4)},
        3);
    if (a.draggable()) {
      g.setFill(a.color());
      g.fillOval(p1.getX() - 4, p1.getY() - 4, 8, 8);
    }
  }

  private static void drawRotated(GraphicsContext g, String text, double x, double y, double deg, Color color) {
    g.save();
    g.translate(x, y);
    g.rotate(deg);
    g.setFill(color);
    g.fillText(text, 0, 0);
    g.restore();
  }

  static Color parseColor(String hex) {
    try {
      return hex == null ? Color.GRAY : Color.web(hex);
    } catch (IllegalArgumentException e) {
      return Color.GRAY;
    }
  }
}
