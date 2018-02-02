#include <mbgl/util/tile_cover.hpp>
#include <mbgl/util/constants.hpp>
#include <mbgl/util/interpolate.hpp>
#include <mbgl/map/transform_state.hpp>

#include <functional>
#include <unordered_map>
#include <utility>

namespace mbgl {

namespace {

// Taken from polymaps src/Layer.js
// https://github.com/simplegeo/polymaps/blob/master/src/Layer.js#L333-L383
struct edge {
    double x0 = 0, y0 = 0;
    double x1 = 0, y1 = 0;
    double dx = 0, dy = 0;

    edge(Point<double> a, Point<double> b) {
        if (a.y > b.y) std::swap(a, b);
        x0 = a.x;
        y0 = a.y;
        x1 = b.x;
        y1 = b.y;
        dx = b.x - a.x;
        dy = b.y - a.y;
    }
};

using ScanLine = const std::function<void(int32_t x0, int32_t x1, int32_t y)>;

// scan-line conversion
static void scanSpans(edge e0, edge e1, int32_t ymin, int32_t ymax, ScanLine scanLine) {
    double y0 = ::fmax(ymin, std::floor(e1.y0));
    double y1 = ::fmin(ymax, std::ceil(e1.y1));

    // sort edges by x-coordinate
    if ((e0.x0 == e1.x0 && e0.y0 == e1.y0) ?
        (e0.x0 + e1.dy / e0.dy * e0.dx < e1.x1) :
        (e0.x1 - e1.dy / e0.dy * e0.dx < e1.x0)) {
        std::swap(e0, e1);
    }

    // scan lines!
    double m0 = e0.dx / e0.dy;
    double m1 = e1.dx / e1.dy;
    double d0 = e0.dx > 0; // use y + 1 to compute x0
    double d1 = e1.dx < 0; // use y + 1 to compute x1
    for (int32_t y = y0; y < y1; y++) {
        double x0 = m0 * ::fmax(0, ::fmin(e0.dy, y + d0 - e0.y0)) + e0.x0;
        double x1 = m1 * ::fmax(0, ::fmin(e1.dy, y + d1 - e1.y0)) + e1.x0;
        scanLine(std::floor(x1), std::ceil(x0), y);
    }
}

// scan-line conversion
static void scanTriangle(const Point<double>& a, const Point<double>& b, const Point<double>& c, int32_t ymin, int32_t ymax, ScanLine& scanLine) {
    edge ab = edge(a, b);
    edge bc = edge(b, c);
    edge ca = edge(c, a);

    // sort edges by y-length
    if (ab.dy > bc.dy) { std::swap(ab, bc); }
    if (ab.dy > ca.dy) { std::swap(ab, ca); }
    if (bc.dy > ca.dy) { std::swap(bc, ca); }

    // scan span! scan span!
    if (ab.dy) scanSpans(ca, ab, ymin, ymax, scanLine);
    if (bc.dy) scanSpans(ca, bc, ymin, ymax, scanLine);
}

} // namespace

namespace util {

namespace {

std::vector<UnwrappedTileID> tileCover(const Point<double>& tl,
                                       const Point<double>& tr,
                                       const Point<double>& br,
                                       const Point<double>& bl,
                                       const Point<double>& c,
                                       int32_t z) {
    const int32_t tiles = 1 << z;

    struct ID {
        int32_t x, y;
        double sqDist;
    };

    std::vector<ID> t;

    auto scanLine = [&](int32_t x0, int32_t x1, int32_t y) {
        int32_t x;
        if (y >= 0 && y <= tiles) {
            for (x = x0; x < x1; ++x) {
                const auto dx = x + 0.5 - c.x, dy = y + 0.5 - c.y;
                t.emplace_back(ID{ x, y, dx * dx + dy * dy });
            }
        }
    };

    // Divide the screen up in two triangles and scan each of them:
    // \---+
    // | \ |
    // +---\.
    scanTriangle(tl, tr, br, 0, tiles, scanLine);
    scanTriangle(br, bl, tl, 0, tiles, scanLine);

    // Sort first by distance, then by x/y.
    std::sort(t.begin(), t.end(), [](const ID& a, const ID& b) {
        return std::tie(a.sqDist, a.x, a.y) < std::tie(b.sqDist, b.x, b.y);
    });

    // Erase duplicate tile IDs (they typically occur at the common side of both triangles).
    t.erase(std::unique(t.begin(), t.end(), [](const ID& a, const ID& b) {
                return a.x == b.x && a.y == b.y;
            }), t.end());

    std::vector<UnwrappedTileID> result;
    for (const auto& id : t) {
        result.emplace_back(z, id.x, id.y);
    }
    return result;
}

Point<double> projectPoint(const Point<double>& pt, int32_t zoom, uint16_t tileSize_) {
    const double worldSize = tileSize_ * std::pow(2, zoom);
    Point<double> projectedPt(
            util::LONGITUDE_MAX + pt.x,
            util::LONGITUDE_MAX - util::RAD2DEG * std::log(std::tan(M_PI / 4 + pt.y * M_PI / util::DEGREES_MAX)));
    projectedPt *= worldSize / util::DEGREES_MAX;
    return projectedPt;
}

Point<double> projectPoint(const LatLng& latLng, int32_t zoom, uint16_t tileSize_) {
    return projectPoint(Point<double>{latLng.longitude(), latLng.latitude()}, zoom, tileSize_);
}

Point<double> pointInTile(Point<double> pt, int32_t zoom, uint16_t tileSize_) {
    auto projectedPoint = projectPoint(pt, zoom, tileSize_);
    return { projectedPoint.x/tileSize_ , projectedPoint.y/tileSize_ };
}

UnwrappedTileID pointToTile(const Point<double>& pt, int32_t zoom, uint16_t tileSize_) {
    const double t2z = tileSize_ * std::pow(2, zoom);
    auto projectedPt = projectPoint(pt, zoom, tileSize_);
    int32_t x = floor((std::min(projectedPt.x, t2z)) / tileSize_);
    int32_t y = floor((std::min(projectedPt.y, t2z)) / tileSize_);
    return UnwrappedTileID{ (uint8_t)zoom, x, y };
}

std::vector<UnwrappedTileID> lineCover(const std::vector<Point<double>>& coords, int32_t zoom) {
    std::unordered_map<UnwrappedTileID, bool> tiles;
    std::vector<UnwrappedTileID> tileIds;

    auto numCoords = coords.size();
    for (uint32_t i=0; i < numCoords - 1; i++) {
        auto startTilePoint = pointInTile(coords[i], zoom, util::tileSize);
        auto stopTilePoint = pointInTile(coords[i+1], zoom, util::tileSize);
        // Use slop of line between two points to determine rate of scan
        auto slope = stopTilePoint - startTilePoint;
        if (slope.x == 0 && slope.y == 0) continue;

        auto dirX = slope.x > 0 ? 1 : -1;
        auto dirY = slope.y > 0 ? 1 : -1;

        Point<double> tilePt = { floor(startTilePoint.x), floor(startTilePoint.y) };

        auto tMaxX = slope.x == 0 ? INT_MAX : std::abs(((slope.x > 0 ? 1 : 0) + tilePt.x - startTilePoint.x)/slope.x);
        auto tMaxY = slope.y == 0 ? INT_MAX : std::abs(((slope.y > 0 ? 1 : 0) + tilePt.y - startTilePoint.y)/ slope.y);

        auto tdx = std::abs(dirX / slope.x);
        auto tdy = std::abs(dirY / slope.y);

        tiles.emplace(std::make_pair(UnwrappedTileID(zoom, tilePt.x, tilePt.y), true));

        while (tMaxX < 1 || tMaxY < 1) {
            if (tMaxX < tMaxY) {
                tMaxX += tdx;
                tilePt.x += dirX;
            } else {
                tMaxY += tdy;
                tilePt.y += dirY;
            }
            tiles.emplace(std::make_pair(UnwrappedTileID(zoom, tilePt.x, tilePt.y), true));
        }
    }
    for(auto &p: tiles) {
        tileIds.push_back(p.first);
    }
    return tileIds;
}

} // namespace

int32_t coveringZoomLevel(double zoom, style::SourceType type, uint16_t size) {
    zoom += std::log(util::tileSize / size) / std::log(2);
    if (type == style::SourceType::Raster || type == style::SourceType::Video) {
        return ::round(zoom);
    } else {
        return std::floor(zoom);
    }
}

std::vector<UnwrappedTileID> tileCover(const LatLngBounds& bounds_, int32_t z) {
    if (bounds_.isEmpty() ||
        bounds_.south() >  util::LATITUDE_MAX ||
        bounds_.north() < -util::LATITUDE_MAX) {
        return {};
    }

    LatLngBounds bounds = LatLngBounds::hull(
        { std::max(bounds_.south(), -util::LATITUDE_MAX), bounds_.west() },
        { std::min(bounds_.north(),  util::LATITUDE_MAX), bounds_.east() });

    return tileCover(
        TileCoordinate::fromLatLng(z, bounds.northwest()).p,
        TileCoordinate::fromLatLng(z, bounds.northeast()).p,
        TileCoordinate::fromLatLng(z, bounds.southeast()).p,
        TileCoordinate::fromLatLng(z, bounds.southwest()).p,
        TileCoordinate::fromLatLng(z, bounds.center()).p,
        z);
}

std::vector<UnwrappedTileID> tileCover(const Geometry<double>& geom, int32_t zoom ) {

    struct ToTileCover {
        int32_t zoom;
        std::vector<UnwrappedTileID> operator()(const Point<double>& g) const {
            return { pointToTile(g, zoom, util::tileSize) };
        }
        std::vector<UnwrappedTileID> operator()(const MultiPoint<double>& g) const {
            std::vector<UnwrappedTileID> _tiles;
            for (auto& pt: g) {
                _tiles.emplace_back(pointToTile(pt, zoom, util::tileSize));
            };
            return _tiles;
        }
        std::vector<UnwrappedTileID> operator()(const LineString<double>& g) const {
            return lineCover(g, zoom);
        }
        std::vector<UnwrappedTileID> operator()(const MultiLineString<double>& g) const {
            std::vector<UnwrappedTileID> tiles;
            for(auto& line: g) {
                auto tileIds = lineCover(line, zoom);
                //TODO: AHM: 
                //tiles.insert(tiles.end(), tileIds.begin(), tileIds.end());
            }
            return tiles;
        }
        std::vector<UnwrappedTileID> operator()(const Polygon<double>& ) const {
            return {};
        }
        std::vector<UnwrappedTileID> operator()(const MultiPolygon<double>& ) const {
            return {};
        }
        std::vector<UnwrappedTileID> operator()(const mapbox::geometry::geometry_collection<double>& ) const {
            return {};
        }
    };
    ToTileCover ttc;
    ttc.zoom = zoom;
    auto tiles = apply_visitor(ttc, geom);
    return tiles;
}

std::vector<UnwrappedTileID> tileCover(const TransformState& state, int32_t z) {
    assert(state.valid());

    const double w = state.getSize().width;
    const double h = state.getSize().height;
    return tileCover(
        TileCoordinate::fromScreenCoordinate(state, z, { 0,   0   }).p,
        TileCoordinate::fromScreenCoordinate(state, z, { w,   0   }).p,
        TileCoordinate::fromScreenCoordinate(state, z, { w,   h   }).p,
        TileCoordinate::fromScreenCoordinate(state, z, { 0,   h   }).p,
        TileCoordinate::fromScreenCoordinate(state, z, { w/2, h/2 }).p,
        z);
}

// Taken from https://github.com/mapbox/sphericalmercator#xyzbbox-zoom-tms_style-srs
// Computes the projected tiles for the lower left and upper right points of the bounds
// and uses that to compute the tile cover count
uint64_t tileCount(const LatLngBounds& bounds, uint8_t zoom, uint16_t tileSize_){

    auto sw = projectPoint(bounds.southwest().wrapped(), zoom, tileSize_);
    auto ne = projectPoint(bounds.northeast().wrapped(), zoom, tileSize_);

    auto x1 = floor(sw.x/ tileSize_);
    auto x2 = floor((ne.x - 1) / tileSize_);
    auto y1 = floor(sw.y/ tileSize_);
    auto y2 = floor((ne.y - 1) / tileSize_);

    auto minX = ::fmax(std::min(x1, x2), 0);
    auto maxX = std::max(x1, x2);
    auto minY = (std::pow(2, zoom) - 1) - std::max(y1, y2);
    auto maxY = (std::pow(2, zoom) - 1) - ::fmax(std::min(y1, y2), 0);
    
    return (maxX - minX + 1) * (maxY - minY + 1);
}

} // namespace util
} // namespace mbgl
