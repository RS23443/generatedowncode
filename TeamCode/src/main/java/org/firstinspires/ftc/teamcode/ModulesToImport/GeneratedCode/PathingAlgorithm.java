package org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode;

import java.util.*;

public class PathingAlgorithm {
    private static final int GRID_SIZE = 145; // Covers -72 to 72
    private static final int OFFSET = 72; // Offset to map world coordinates to grid indices
    private static final int[][] grid = new int[GRID_SIZE][GRID_SIZE]; // Preset grid

    static {
        // Define rectangle bounds in world coordinates
        double left = -14;
        double right = 14;
        double top = 24;
        double bottom = -24;

        // Convert bounds to grid indices
        int leftX = (int) Math.round(left + OFFSET);
        int rightX = (int) Math.round(right + OFFSET);
        int topY = (int) Math.round(top + OFFSET);
        int bottomY = (int) Math.round(bottom + OFFSET);

        // Mark edges as obstacles
        for (int x = leftX; x <= rightX; x++) {
            grid[x][topY] = 1;    // Top edge
            grid[x][bottomY] = 1; // Bottom edge
        }
        for (int y = bottomY; y <= topY; y++) {
            grid[leftX][y] = 1;   // Left edge
            grid[rightX][y] = 1;  // Right edge
        }
    }

    // Inner Point class to handle grid coordinates and heading
    public static class Point {
        int x, y;
        double heading;

        public Point(int x, int y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        // Convert world coordinates to grid indices
        public static Point toGridIndex(double worldX, double worldY) {
            int gridX = (int) Math.round(worldX + OFFSET);
            int gridY = (int) Math.round(worldY + OFFSET);
            return new Point(gridX, gridY, 0); // Default heading as 0
        }

        // Convert grid indices back to world coordinates
        public double[] toWorldCoordinate() {
            double worldX = x - OFFSET;
            double worldY = y - OFFSET;
            return new double[]{worldX, worldY};
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            Point point = (Point) obj;
            return x == point.x && y == point.y;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y);
        }
    }

    // A* Algorithm Implementation
    public static List<Point> findPath(Point start, Point goal) {
        PriorityQueue<Point> openSet = new PriorityQueue<>(Comparator.comparingInt(p -> heuristic(p, goal)));
        Set<Point> closedSet = new HashSet<>();
        Map<Point, Point> cameFrom = new HashMap<>();
        Map<Point, Integer> gScore = new HashMap<>();

        gScore.put(start, 0);
        openSet.add(start);

        while (!openSet.isEmpty()) {
            Point current = openSet.poll();

            // If we reach the goal, reconstruct the path
            if (current.equals(goal)) {
                return reconstructPath(cameFrom, current);
            }

            closedSet.add(current);

            // Explore neighbors
            for (Point neighbor : getNeighbors(current)) {
                if (closedSet.contains(neighbor)) continue;

                int tentativeGScore = gScore.getOrDefault(current, Integer.MAX_VALUE) + 1; // Movement cost = 1

                if (tentativeGScore < gScore.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                    cameFrom.put(neighbor, current);
                    gScore.put(neighbor, tentativeGScore);

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }

        return Collections.emptyList(); // Return an empty list if no path is found
    }

    // Heuristic function for A* (Manhattan Distance)
    private static int heuristic(Point current, Point goal) {
        return Math.abs(current.x - goal.x) + Math.abs(current.y - goal.y);
    }

    // Get neighbors for grid movement (4 directions)
    private static List<Point> getNeighbors(Point point) {
        List<Point> neighbors = new ArrayList<>();
        int[][] directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}; // Right, left, up, down

        for (int[] dir : directions) {
            int nx = point.x + dir[0];
            int ny = point.y + dir[1];
            if (isValid(nx, ny)) {
                double heading = Math.atan2(dir[1], dir[0]);
                neighbors.add(new Point(nx, ny, heading));
            }
        }

        return neighbors;
    }

    // Validate if a grid cell is walkable
    private static boolean isValid(int x, int y) {
        return x >= 0 && y >= 0 && x < GRID_SIZE && y < GRID_SIZE && grid[x][y] == 0; // 0 = walkable
    }

    // Reconstruct the path from goal to start
    private static List<Point> reconstructPath(Map<Point, Point> cameFrom, Point current) {
        List<Point> path = new ArrayList<>();
        Point next = null;

        while (current != null) {
            if (next != null) {
                // Calculate heading from current to next
                double[] currentWorld = current.toWorldCoordinate();
                double[] nextWorld = next.toWorldCoordinate();
                double dx = nextWorld[0] - currentWorld[0];
                double dy = nextWorld[1] - currentWorld[1];
                current.heading = Math.atan2(dy, dx);
            }

            path.add(current);
            next = current;
            current = cameFrom.get(current);
        }

        Collections.reverse(path);
        return path;
    }
}