#include "MapRenderer.h"

#include <algorithm>
#include <cmath>

#include "Geo.h"

namespace logi {

namespace {
const Color kBackground{24, 28, 36, 255};
const Color kRoad{92, 102, 120, 255};
const Color kNode{150, 160, 178, 255};
const Color kRoute{255, 196, 61, 255};
}  // namespace

MapRenderer::MapRenderer(const RoadNetwork& network) : network_(network) {
    const BoundingBox b = network_.bounds();  // throws if the network is empty
    origin_ = {(b.south + b.north) / 2.0, (b.west + b.east) / 2.0};
    cosOriginLat_ = std::cos(toRadians(origin_.lat));
}

// Equirectangular projection around origin_: accurate to well under 1% for
// city-sized areas, and trivially invertible for mouse picking.
//   x = R * dLon * cos(lat0)      y = -R * dLat   (screen y grows downward)
Vector2 MapRenderer::toMeters(const GeoPoint& p) const {
    const double x = kEarthRadiusMeters * toRadians(p.lon - origin_.lon) * cosOriginLat_;
    const double y = -kEarthRadiusMeters * toRadians(p.lat - origin_.lat);
    return {static_cast<float>(x), static_cast<float>(y)};
}

Vector2 MapRenderer::toScreen(const GeoPoint& p) const {
    const Vector2 m = toMeters(p);
    return {m.x * scale_ + offset_.x, m.y * scale_ + offset_.y};
}

GeoPoint MapRenderer::toGeo(Vector2 s) const {
    const double mx = (s.x - offset_.x) / scale_;
    const double my = (s.y - offset_.y) / scale_;
    return {origin_.lat + toDegrees(-my / kEarthRadiusMeters),
            origin_.lon + toDegrees(mx / (kEarthRadiusMeters * cosOriginLat_))};
}

void MapRenderer::fitToWindow(int width, int height) {
    const BoundingBox b = network_.bounds();
    const Vector2 a = toMeters({b.south, b.west});
    const Vector2 c = toMeters({b.north, b.east});
    const float spanX = std::max(1.0f, std::fabs(c.x - a.x));
    const float spanY = std::max(1.0f, std::fabs(c.y - a.y));
    const float margin = 40.0f;
    scale_ = std::min((width - 2 * margin) / spanX, (height - 2 * margin) / spanY);
    offset_ = {width / 2.0f, height / 2.0f};  // origin_ is the centre of the box
    cacheDirty_ = true;
}

void MapRenderer::zoomAt(Vector2 p, float wheel) {
    const float factor = std::pow(1.15f, wheel);
    scale_ *= factor;
    // Keep the point under the cursor fixed while scaling.
    offset_ = {p.x - (p.x - offset_.x) * factor, p.y - (p.y - offset_.y) * factor};
    cacheDirty_ = true;
}

void MapRenderer::pan(Vector2 d) {
    offset_ = {offset_.x + d.x, offset_.y + d.y};
    cacheDirty_ = true;
}

void MapRenderer::refreshScreenCache() {
    if (!cacheDirty_) return;
    screen_.resize(network_.nodeCount());
    for (RoadNetwork::NodeId i = 0; i < network_.nodeCount(); ++i) screen_[i] = toScreen(network_.position(i));
    cacheDirty_ = false;
}

void MapRenderer::draw(const DeliveryManager& dm) {
    refreshScreenCache();
    ClearBackground(kBackground);

    // 1. Roads = lines. Each undirected road is stored twice, so draw only u < v.
    for (RoadNetwork::NodeId u = 0; u < network_.nodeCount(); ++u) {
        for (const auto& e : network_.neighbors(u)) {
            if (u < e.first) DrawLineV(screen_[u], screen_[e.first], kRoad);
        }
    }

    // 2. Nodes = circles. On big maps only junctions/dead ends (degree != 2) to avoid clutter.
    const bool drawAll = network_.nodeCount() < 4000;
    for (RoadNetwork::NodeId u = 0; u < network_.nodeCount(); ++u) {
        if (drawAll || network_.neighbors(u).size() != 2) DrawCircleV(screen_[u], 2.0f, kNode);
    }

    // 2b. Named OpenStreetMap places. Keep labels at readable zoom levels.
    if (scale_ > 0.00002f) {
        for (const auto& place : places_) {
            const Vector2 p = toScreen(place.position);
            if (p.x < -100 || p.y < -30 || p.x > GetScreenWidth() + 100 || p.y > GetScreenHeight() + 30) continue;
            DrawCircleV(p, 2.5f, Color{110, 210, 220, 255});
            DrawText(place.name.c_str(), static_cast<int>(p.x) + 5, static_cast<int>(p.y) - 13, 11,
                     Color{205, 225, 235, 255});
        }
    }

    // 3. Planned A* route.
    const auto& route = dm.routeNodes();
    for (std::size_t i = 1; i < route.size(); ++i) {
        DrawLineEx(screen_[route[i - 1]], screen_[route[i]], 3.0f, kRoute);
    }

    // 4. Depot (blue square) and customers (red = pending, green = delivered, numbered in order).
    const Vector2 depot = screen_[dm.depot()];
    DrawRectangleV({depot.x - 8, depot.y - 8}, {16, 16}, Color{64, 140, 255, 255});
    DrawText("DEPOT", static_cast<int>(depot.x) + 12, static_cast<int>(depot.y) - 8, 14, RAYWHITE);
    const auto& customers = dm.customers();
    for (std::size_t i = 0; i < customers.size(); ++i) {
        const Vector2 p = screen_[customers[i]];
        const bool done = i < dm.deliveredCount();
        DrawCircleV(p, 8.0f, done ? Color{80, 200, 120, 255} : Color{235, 70, 70, 255});
        DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y), 8.0f, RAYWHITE);
        DrawText(TextFormat("%d", static_cast<int>(i + 1)), static_cast<int>(p.x) + 11,
                 static_cast<int>(p.y) - 8, 14, RAYWHITE);
    }

    // 5. Truck.
    drawTruck(dm.truckPose());
}

// The truck is built from primitives: body + cab rectangles and 4 wheels, all
// rotated to the heading of the road segment it is currently driving along.
void MapRenderer::drawTruck(const DeliveryManager::TruckPose& pose) {
    const Vector2 p = toScreen(pose.position);
    const Vector2 a = toScreen(pose.segmentFrom);
    const Vector2 b = toScreen(pose.segmentTo);
    if (std::hypot(b.x - a.x, b.y - a.y) > 1e-3f) lastHeadingRad_ = std::atan2(b.y - a.y, b.x - a.x);

    const float h = lastHeadingRad_;
    const Vector2 dir{std::cos(h), std::sin(h)};
    const Vector2 perp{-dir.y, dir.x};
    const float deg = h * 180.0f / static_cast<float>(kPi);

    DrawRectanglePro({p.x, p.y, 22.0f, 11.0f}, {11.0f, 5.5f}, deg, Color{245, 245, 245, 255});  // cargo box
    DrawRectanglePro({p.x + dir.x * 14.5f, p.y + dir.y * 14.5f, 8.0f, 9.0f}, {4.0f, 4.5f}, deg,
                     Color{230, 90, 40, 255});  // cab
    for (float along : {-6.0f, 6.0f}) {
        for (float side : {-6.0f, 6.0f}) {
            DrawCircleV({p.x + dir.x * along + perp.x * side, p.y + dir.y * along + perp.y * side}, 2.2f, BLACK);
        }
    }
}

void MapRenderer::drawHud(const DeliveryManager& dm, const std::string& status) const {
    DrawRectangle(8, 8, 430, 150, Fade(BLACK, 0.65f));
    DrawText("Logistics Router  (A* + OpenStreetMap)", 18, 14, 18, RAYWHITE);
    DrawText(TextFormat("State: %s%s", toString(dm.state()), dm.paused() ? " [paused]" : ""), 18, 40, 16, RAYWHITE);
    DrawText(TextFormat("Delivered: %d / %d   Route: %.2f km", static_cast<int>(dm.deliveredCount()),
                        static_cast<int>(dm.customers().size()), dm.totalPlannedMeters() / 1000.0),
             18, 60, 16, RAYWHITE);
    DrawText(TextFormat("Graph: %d nodes, %d roads   Speed x%.0f   Labels: %d",
                        static_cast<int>(network_.nodeCount()), static_cast<int>(network_.edgeCount()),
                        dm.timeScale(), static_cast<int>(places_.size())),
             18, 80, 16, RAYWHITE);
    DrawText("SPACE start/pause  R reset  O nearest-first  C clear  S save  N named destination", 18, 104, 13, LIGHTGRAY);
    DrawText("Left-click: add  Right-drag: pan  Wheel: zoom  Up/Down: speed", 18, 122, 13, LIGHTGRAY);
    if (!status.empty()) DrawText(status.c_str(), 18, GetScreenHeight() - 28, 16, Color{255, 210, 120, 255});
    DrawFPS(GetScreenWidth() - 90, 10);
}

}  // namespace logi
