#include "lest.hpp"
#include "MarchingCubes.h"
#include "original-implementation/MarchingCubes.h"

template <typename MC, typename F>
void SetupMarchingCubes(MC &mc, int size, F f)
{
    float xmin = -1.0f, xmax = 1.0f, ymin = -1.0f, ymax = 1.0f, zmin = -1.0f,
          zmax = 1.0f;

    float rx = (xmax - xmin) / (size - 1);
    float ry = (ymax - ymin) / (size - 1);
    float rz = (zmax - zmin) / (size - 1);
    glm::vec3 min_pos(xmin, ymin, zmin);
    glm::vec3 range(rx, ry, rz);
    for (int i = 0; i < size; i++) {
        auto x = (float)i * rx + xmin;
        for (int j = 0; j < size; j++) {
            auto y = (float)j * ry + ymin;
            for (int k = 0; k < size; k++) {
                auto z = (float)k * rz + zmin;

                auto pos = glm::vec3(x, y, z);
                // auto w =
                auto w = f(pos);
                mc.set_data(w, i, j, k);
            }
        }
    }
}

float SphereDistanceField(const glm::vec3 &pos)
{
    return glm::length(pos) - 0.49f;
}

template <typename F>
void TestMarchingCubes(lest::env &lest_env, int size, F f)
{
    auto mc_new = MarchingCubes(glm::ivec3(size));
    auto mc_old = original::MarchingCubes();

    // new-style setup
    mc_new.Setup();

    // old-style setup
    mc_old.clean_all();
    mc_old.set_resolution(size, size, size);
    mc_old.init_all();

    SetupMarchingCubes(mc_new, size, SphereDistanceField);
    SetupMarchingCubes(mc_old, size, SphereDistanceField);

    auto isolevel = 0.f;
    mc_old.compute_intersection_points(isolevel);
    mc_new.compute_intersection_points(isolevel);

    for (int k = 0; k < size; ++k) {
        for (int j = 0; j < size; ++j) {
            for (int i = 0; i < size; ++i) {
                EXPECT(mc_old.get_x_vert(i, j, k) ==
                       mc_new.get_x_vert(glm::ivec3(i, j, k)));
                EXPECT(mc_old.get_y_vert(i, j, k) ==
                       mc_new.get_y_vert(glm::ivec3(i, j, k)));
                EXPECT(mc_old.get_z_vert(i, j, k) ==
                       mc_new.get_z_vert(glm::ivec3(i, j, k)));
            }
        }
    }

    EXPECT(mc_old.nverts() == mc_new.nverts());
    for (int i = 0; i < mc_old.nverts(); ++i) {
        auto v_old = mc_old.vert(i);
        auto v_new = mc_new.vert(i);
        EXPECT(v_old->x == lest::approx(v_new.pos.x));
        EXPECT(v_old->y == lest::approx(v_new.pos.y));
        EXPECT(v_old->z == lest::approx(v_new.pos.z));
    }

    EXPECT(mc_old.ntrigs() == mc_new.ntrigs());
    for (int i = 0; i < mc_old.ntrigs(); ++i) {
        auto t_old = mc_old.trig(i);
        auto t_new = mc_new.trig(i);
        EXPECT(t_old->v1 == t_new.ids.x);
        EXPECT(t_old->v2 == t_new.ids.y);
        EXPECT(t_old->v3 == t_new.ids.z);
    }
}

const lest::test specification[] = {
    CASE("Empty string has length zero (succeed)"){
        EXPECT(0 == std::string().length());
EXPECT(0 == std::string("").length());
}
, CASE("Test MC")
{
    auto size = int{50};
    TestMarchingCubes(lest_env, size, SphereDistanceField);
}
}
;

int main(int argc, char *argv[])
{
    return lest::run(specification, argc, argv);
}
