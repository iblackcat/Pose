#include "stereo_matching.h"

namespace mf {

StereoMatching::StereoMatching()
{
}


StereoMatching::~StereoMatching()
{
}

bool StereoMatching::init(int method) {
	char method_file_name[256];
	switch (method) {

	case STEREOMATCH_SSD:
		sprintf(method_file_name, "shader/stereo_matching_SSD.frag");
		break;
	
	case STEREOMATCH_ZNCC:
		sprintf(method_file_name, "shader/stereo_matching_ZNCC.frag");
		break;
	
	}
	
	//disparity estimation renderer
	//input: u32, u32; output: float
	if (!m_gl_matching.init(G.w, G.h, GL_R32F, GL_RED, GL_FLOAT))
		return false;
	if (!m_gl_matching.setShaderFile("shader/default.vert", method_file_name))
		return false;
	m_gl_matching.CreateVertexBuffer();
	m_gl_matching_tex1 = m_gl_matching.CreateTexture(&m_gl_matching_tex1.tex_id, G.w, G.h);
	m_gl_matching_tex2 = m_gl_matching.CreateTexture(&m_gl_matching_tex2.tex_id, G.w, G.h);

	//LR-check and depth renderer
	//input: float, float; output: u8
	if (!m_gl_triangulation.init(G.w, G.h, GL_R8, GL_RED, GL_UNSIGNED_BYTE))
		return false;
	if (!m_gl_triangulation.setShaderFile("shader/default.vert", "shader/lrcheck_and_triangulation.frag"))
		return false;
	m_gl_triangulation.CreateVertexBuffer();
	m_gl_triangulation_tex1 = m_gl_triangulation.CreateTexture(&m_gl_triangulation_tex1.tex_id, G.w, G.h, GL_R32F, GL_RED, GL_FLOAT);
	m_gl_triangulation_tex2 = m_gl_triangulation.CreateTexture(&m_gl_triangulation_tex2.tex_id, G.w, G.h, GL_R32F, GL_RED, GL_FLOAT);

	return true;
}

bool StereoMatching::destroy() {
	m_gl_matching.destroy();
	m_gl_triangulation.destroy();

	return true;
}

void StereoMatching::disparity_estimation(const u32 const *image1, const u32 const *image2, float **delta1, float **delta2, const int& radius) {
	float *d1 = nullptr, *d2 = nullptr;

	m_gl_matching.useRenderer();
	
	m_gl_matching.setUniform1("m_w", G.w);
	m_gl_matching.setUniform1("m_h", G.h);
	m_gl_matching.setUniform1("step", static_cast<int>(1));
	m_gl_matching.setUniform1("radius", static_cast<int>(radius));
	m_gl_matching.setTexSub2D("tex", m_gl_matching_tex1, 0, GL_TEXTURE0, image1);
	m_gl_matching.setTexSub2D("tex2", m_gl_matching_tex2, 1, GL_TEXTURE1, image2);
	d1 = m_gl_matching.RenderScence<float>();

	m_gl_matching.setUniform1("m_w", G.w);
	m_gl_matching.setUniform1("m_h", G.h);
	m_gl_matching.setUniform1("step", static_cast<int>(-1));
	m_gl_matching.setUniform1("radius", static_cast<int>(radius));
	m_gl_matching.setTexSub2D("tex", m_gl_matching_tex1, 0, GL_TEXTURE0, image2);
	m_gl_matching.setTexSub2D("tex2", m_gl_matching_tex2, 1, GL_TEXTURE1, image1);
	d2 = m_gl_matching.RenderScence<float>();

	*delta1 = d1;
	*delta2 = d2;
}


u8* StereoMatching::lrcheck_and_depth(const float const *delta1, const float const *delta2, float baseline, int max_diff) {
	u8 *depth = nullptr;

	m_gl_triangulation.useRenderer();

	m_gl_triangulation.setUniform1("m_w", static_cast<int>(G.w));
	m_gl_triangulation.setUniform1("max_diff", static_cast<int>(max_diff));
	m_gl_triangulation.setUniform1("baseline", static_cast<float>(baseline));
	m_gl_triangulation.setUniform1("fx", static_cast<float>(G.fx));

	m_gl_triangulation.setTexSub2D("tex", m_gl_triangulation_tex1, 0, GL_TEXTURE0, delta1);
	m_gl_triangulation.setTexSub2D("tex2", m_gl_triangulation_tex2, 1, GL_TEXTURE1, delta2);

	depth = m_gl_triangulation.RenderScence<u8>();
	return depth;
}

u8* StereoMatching::stereo_matching(const u32 *image1, const u32 *image2, float baseline, int max_diff, int radius) {
	float *delta1 = nullptr, *delta2 = nullptr;
	u8 *depth = nullptr;

	disparity_estimation(image1, image2, &delta1, &delta2, radius);
	depth = lrcheck_and_depth(delta1, delta2, baseline, max_diff);

	if (delta1) { free(delta1); delta1 = nullptr; }
	if (delta2) { free(delta2); delta2 = nullptr; }
	return depth;
}

u8* StereoMatching::stereo_matching(const u32 *image1, const CameraPose &p1, const u32 *image2, const CameraPose &p2, int max_diff, int radius) {
	float *delta1 = nullptr, *delta2 = nullptr;
	u8 *depth = nullptr;

	float baseline = static_cast<float>((p1.center - p2.center).norm());
	std::cout << "baseline: " << baseline << std::endl;

	disparity_estimation(image1, image2, &delta1, &delta2, radius);
	depth = lrcheck_and_depth(delta1, delta2, baseline, max_diff);

	if (delta1) { free(delta1); delta1 = nullptr; }
	if (delta2) { free(delta2); delta2 = nullptr; }
	return depth;
}

} // namespace mf
