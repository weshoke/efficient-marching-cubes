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
	void mouseDown( MouseEvent event ) override;
	void mouseDrag( MouseEvent event ) override;
	void resize() override;
	void update() override;
	void draw() override;
	
	
	CameraPersp camera;
	CameraUi camUi;
	std::vector<gl::BatchRef> batches;
};

gl::GlslProgRef LoadShader(const std::string& name, bool has_geometry_shader=true) {
	gl::GlslProgRef shader;
	try {
		auto format = gl::GlslProg::Format()
		.vertex(loadAsset(name+".vert"))
		.fragment(loadAsset(name+".frag"));
		
		if(has_geometry_shader) {
			format = format.geometry(loadAsset(name+".geom"));
		}
		
		shader = gl::GlslProg::create(format);
	}
	catch(Exception &exc ) {
		CI_LOG_E("error loading " << name << " shader: " << exc.what());
	}
	return shader;
}

void MarchingCubesApp::setup()
{
	camera.lookAt(normalize(vec3(0, 0, 1)) * 5.0f, vec3(0.f, 0.f, 0.f));
	camUi = CameraUi(&camera);
	ci::app::getWindow()->setTitle("Marching Cubes");


	run();

	auto verts = std::vector<glm::vec3>();
	verts.reserve(mc.ntrigs() * 3);

	for( int i = 0 ; i < mc.ntrigs() ; ++i ) {
		const Triangle *t = mc.trig(i) ;
		const Vertex   *v ;
		v = mc.vert(t->v1) ;
		verts.push_back(glm::vec3(v->x, v->y, v->z));
		v = mc.vert(t->v2) ;
		verts.push_back(glm::vec3(v->x, v->y, v->z));
		v = mc.vert(t->v3) ;
		verts.push_back(glm::vec3(v->x, v->y, v->z));
	}
	
	auto mesh = gl::VboMesh::create(verts.size(), GL_LINES, {gl::VboMesh::Layout().attrib(geom::POSITION, 3)});
	mesh->bufferAttrib(geom::POSITION, verts);
	auto shader = LoadShader("pass", false);
	batches.push_back(gl::Batch::create(mesh, shader));
}

void MarchingCubesApp::mouseDown(MouseEvent event) {
	camUi.mouseDown(event);
}

void MarchingCubesApp::mouseDrag(MouseEvent event) {
	camUi.mouseDrag(event);
}

void MarchingCubesApp::resize() {
	camera.setAspectRatio(getWindowAspectRatio());
}

void MarchingCubesApp::update() {
}


void MarchingCubesApp::draw()
{
	//gl::clear( Color( 0, 0, 0 ) );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	
	gl::clear(Color(0.93, 0.93, 0.93));
	gl::setMatrices(camera);
	
	gl::color(1., 0., 0.);
	for(auto& batch : batches) {
		batch->draw();
	}
}

CINDER_APP( MarchingCubesApp, RendererGl )
