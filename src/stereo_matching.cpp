#include "stereo_matching.h"

namespace mf {

StereoMatching::StereoMatching()
{
}


StereoMatching::~StereoMatching()
{
}

bool StereoMatching::init(int method) {
	char method_file[256];
	switch (method) {

	case STEREOMATCH_SSD:
		sprintf(method_file, "shader/stereo_matching_SSD.frag");
	
	case STEREOMATCH_ZNCC:
		sprintf(method_file, "shader/stereo_matching_ZNCC.frag");
	
	}
	
	//disparity estimation renderer
	//input: u32, u32; output: float
	if (!m_gl_matching.init(G.w, G.h, 1, GL_FLOAT))
		return false;
	if (!m_gl_matching.setShaderFile("shader/default.vert", method_file))
		return false;
	m_gl_matching.CreateVertexBuffer();
	m_gl_matching_tex1 = m_gl_matching.CreateTexture(&m_gl_matching_tex1.tex_id, G.w, G.h);
	m_gl_matching_tex2 = m_gl_matching.CreateTexture(&m_gl_matching_tex2.tex_id, G.w, G.h);

	//LR-check and depth renderer
	//input: float, float; output: u8
	if (!m_gl_triangulation.init(G.w, G.h, 1, GL_UNSIGNED_BYTE))
		return false;
	if (!m_gl_triangulation.setShaderFile("shader/default.vert", "shader/lrcheck_and_triangulation.frag"))
		return false;
	m_gl_triangulation.CreateVertexBuffer();
	m_gl_triangulation_tex1 = m_gl_triangulation.CreateTexture(&m_gl_triangulation_tex1.tex_id, G.w, G.h, GL_R, GL_FLOAT);
	m_gl_triangulation_tex2 = m_gl_triangulation.CreateTexture(&m_gl_triangulation_tex2.tex_id, G.w, G.h, GL_R, GL_FLOAT);


}

void StereoMatching::disparity_estimation(const u32 const *image1, const u32 const *image2, float **delta1, float **delta2, const int& radius) {
	float *d1 = nullptr, *d2 = nullptr;

	m_gl_matching.useRenderer();
	m_gl_matching.setUniform1("m_w", G.w);
	m_gl_matching.setUniform1("m_h", G.h);
	m_gl_matching.setUniform1("step", 1);
	m_gl_matching.setUniform1("radius", radius);
	m_gl_matching.setTexSub2D("tex", m_gl_matching_tex1, 0, GL_TEXTURE0, image1);
	m_gl_matching.setTexSub2D("tex2", m_gl_matching_tex2, 1, GL_TEXTURE1, image2);
	d1 = m_gl_matching.RenderScence<float>();

	m_gl_matching.setUniform1("m_w", G.w);
	m_gl_matching.setUniform1("m_h", G.h);
	m_gl_matching.setUniform1("step", -1);
	m_gl_matching.setUniform1("radius", radius);
	m_gl_matching.setTexSub2D("tex", m_gl_matching_tex1, 0, GL_TEXTURE0, image2);
	m_gl_matching.setTexSub2D("tex2", m_gl_matching_tex2, 1, GL_TEXTURE1, image1);
	d2 = m_gl_matching.RenderScence<float>();

	*delta1 = d1;
	*delta2 = d2;
}

/*
u32* StereoMatching::lrcheck_and_depth() {

}
*/

u8* StereoMatching::stereo_matching(const u32 *image1, const CameraPose &p1, const u32 *image2, const CameraPose &p2) {
	u8 *depth = nullptr;
}

} // namespace mf
