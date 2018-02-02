#include <mbgl/util/tile_cover.hpp>
#include <mbgl/util/geo.hpp>
#include <mbgl/map/transform.hpp>

#include <gtest/gtest.h>

using namespace mbgl;

TEST(TileCover, Empty) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{}), util::tileCover(LatLngBounds::empty(), 0));
}

TEST(TileCover, Arctic) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{}),
              util::tileCover(LatLngBounds::hull({ 86, -180 }, { 90, 180 }), 0));
}

TEST(TileCover, Antarctic) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{}),
              util::tileCover(LatLngBounds::hull({ -86, -180 }, { -90, 180 }), 0));
}

TEST(TileCover, WorldZ0) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{
                  { 0, 0, 0 },
              }),
              util::tileCover(LatLngBounds::world(), 0));
}

TEST(TileCover, Pitch) {
    Transform transform;
    transform.resize({ 512, 512 });
    // slightly offset center so that tile order is better defined
    transform.setLatLng({ 0.1, -0.1 });
    transform.setZoom(2);
    transform.setAngle(5.0);
    transform.setPitch(40.0 * M_PI / 180.0);

    EXPECT_EQ((std::vector<UnwrappedTileID>{
                  { 2, 1, 2 }, { 2, 1, 1 }, { 2, 2, 2 }, { 2, 2, 1 }, { 2, 3, 2 }
              }),
              util::tileCover(transform.getState(), 2));
}

TEST(TileCover, WorldZ1) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{
                  { 1, 0, 0 }, { 1, 0, 1 }, { 1, 1, 0 }, { 1, 1, 1 },
              }),
              util::tileCover(LatLngBounds::world(), 1));
}

TEST(TileCover, SingletonZ0) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{}),
              util::tileCover(LatLngBounds::singleton({ 0, 0 }), 0));
}

TEST(TileCover, SingletonZ1) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{}),
              util::tileCover(LatLngBounds::singleton({ 0, 0 }), 1));
}

static const LatLngBounds sanFrancisco =
    LatLngBounds::hull({ 37.6609, -122.5744 }, { 37.8271, -122.3204 });

TEST(TileCover, SanFranciscoZ0) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{
                  { 0, 0, 0 },
              }),
              util::tileCover(sanFrancisco, 0));
}

TEST(TileCover, SanFranciscoZ10) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{
                  { 10, 163, 395 }, { 10, 163, 396 }, { 10, 164, 395 }, { 10, 164, 396 },

              }),
              util::tileCover(sanFrancisco, 10));
}

static const LatLngBounds sanFranciscoWrapped =
    LatLngBounds::hull({ 37.6609, 238.5744 }, { 37.8271, 238.3204 });

TEST(TileCover, SanFranciscoZ0Wrapped) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{ { 0, 1, 0 } }),
              util::tileCover(sanFranciscoWrapped, 0));
}

TEST(TileCover, GeomPointZ13) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{ { 13, 2343, 3133 } }),
            util::tileCover(Point<double> {-77.03355114851098,38.89224995264726 }, 13));
}

TEST(TileCover, GeomPointZ10) {
    EXPECT_EQ((std::vector<UnwrappedTileID>{ { 10, 292, 391 } }),
            util::tileCover(Point<double> {-77.03355114851098,38.89224995264726 }, 10));
}

TEST(TileCover, GeomLineZ13) {
    auto lineCover = util::tileCover(LineString<double>{
            {-77.03342914581299,38.892101707724315},
            {-77.02394485473633,38.89203490311832},
            {-77.02390193939209,38.8824811975508},
            {-77.0119285583496,38.8824811975508},
            {-77.01218605041504,38.887391829071106},
            {-77.01390266418456,38.88735842456116},
            {-77.01622009277342,38.896510672795266},
            {-77.01725006103516,38.914143795902376},
            {-77.01879501342773,38.914143795902376},
            {-77.0196533203125,38.91307524644972}
        }, 13);
    EXPECT_EQ((std::vector<UnwrappedTileID>{ { 13, 2343, 3134 }, { 13, 2343, 3133 } }),
        lineCover);
}

TEST(TileCover, GeomLineZ15) {
    auto lineCover = util::tileCover(LineString<double>{
            {-77.03342914581299,38.892101707724315},
            {-77.02394485473633,38.89203490311832},
            {-77.02390193939209,38.8824811975508},
            {-77.0119285583496,38.8824811975508},
            {-77.01218605041504,38.887391829071106},
            {-77.01390266418456,38.88735842456116},
            {-77.01622009277342,38.896510672795266},
            {-77.01725006103516,38.914143795902376},
            {-77.01879501342773,38.914143795902376},
            {-77.0196533203125,38.91307524644972}
        }, 15);
    EXPECT_EQ(lineCover, (std::vector<UnwrappedTileID>{
        { 15,9373,12534 },
        { 15,9374,12536 },
        { 15,9372,12535 },
        { 15,9373,12537 },
        { 15,9373,12536 },
        { 15,9374,12537 },
        { 15,9373,12533 },
        { 15,9373,12535 }
        }));
}

TEST(TileCount, SanFranciscoZ10) {
    EXPECT_EQ(4u, util::tileCount(sanFrancisco, 10, util::tileSize));
}

TEST(TileCount, SanFranciscoZ22) {
    EXPECT_EQ(7254450u, util::tileCount(sanFrancisco, 22, util::tileSize));
}

