#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "MarchingCubes.h"
#include "glui_defs.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class MarchingCubesApp : public App {
   public:
    void setup() override;
    void mouseDown(MouseEvent event) override;
    void mouseDrag(MouseEvent event) override;
    void resize() override;
    void update() override;
    void draw() override;

    CameraPersp camera;
    CameraUi camUi;
    std::vector<gl::BatchRef> batches;
    gl::GlslProgRef wireframe_shader;
};

gl::GlslProgRef LoadShader(const std::string &name,
                           bool has_geometry_shader = true)
{
    gl::GlslProgRef shader;
    try {
        auto format = gl::GlslProg::Format()
                          .vertex(loadAsset(name + ".vert"))
                          .fragment(loadAsset(name + ".frag"));

        if (has_geometry_shader) {
            format = format.geometry(loadAsset(name + ".geom"));
        }

        shader = gl::GlslProg::create(format);
    }
    catch (Exception &exc) {
        CI_LOG_E("error loading " << name << " shader: " << exc.what());
    }
    return shader;
}

void MarchingCubesApp::setup()
{
    camera.lookAt(normalize(vec3(0, 0, 1)) * 5.0f, vec3(0.f, 0.f, 0.f));
    camUi = CameraUi(&camera);
    ci::app::getWindow()->setTitle("Marching Cubes");

    MarchingCubes mc(glm::ivec3(50));
    mc.Setup();
    run(mc);

    auto verts = std::vector<glm::vec3>();
    verts.reserve(mc.ntrigs() * 3);

    for (int i = 0; i < mc.ntrigs(); ++i) {
        auto tri = mc.trig(i);
        for (int t = 0; t < 3; ++t) {
            verts.push_back(mc.vert(tri.ids[t]).pos);
        }
    }

    auto mesh =
        gl::VboMesh::create(verts.size(), GL_TRIANGLES,
                            {gl::VboMesh::Layout().attrib(geom::POSITION, 3)});
    mesh->bufferAttrib(geom::POSITION, verts);
    // auto shader = LoadShader("pass", false);
    wireframe_shader = LoadShader("wireframe", true);
    wireframe_shader->uniform("uBrightness", 1.f);
    batches.push_back(gl::Batch::create(mesh, wireframe_shader));
}

void MarchingCubesApp::mouseDown(MouseEvent event) { camUi.mouseDown(event); }
void MarchingCubesApp::mouseDrag(MouseEvent event) { camUi.mouseDrag(event); }
void MarchingCubesApp::resize()
{
    camera.setAspectRatio(getWindowAspectRatio());
    wireframe_shader->uniform("uViewportSize",
                              glm::vec2(toPixels(getWindowSize())));
}

void MarchingCubesApp::update() {}
void MarchingCubesApp::draw()
{
    // gl::clear( Color( 0, 0, 0 ) );
    gl::enableDepthRead();
    gl::enableDepthWrite();

    gl::clear(Color(0.93, 0.93, 0.93));
    gl::setMatrices(camera);

    // gl::color(0.124 * 0.7, 0.37, 0.36);
    // gl::color(0.93, 0.93, 0.93);
    gl::color(0.98, 0.98, 0.98);
    for (auto &batch : batches) {
        batch->draw();
    }
}

CINDER_APP(MarchingCubesApp, RendererGl)
