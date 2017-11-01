#pragma once

#include "utils.h"
#include "gl_helper.h"

namespace mf {

class StereoMatching
{
public:
	StereoMatching();
	~StereoMatching();

	enum {
		STEREOMATCH_SSD = 0,
		STEREOMATCH_ZNCC = 1
	};

	bool init(int method = STEREOMATCH_SSD);
	u8* stereo_matching(const u32 *image1, const CameraPose &p1, const u32 *image2, const CameraPose &p2);
	void StereoMatching::disparity_estimation(const u32 const *image1, const u32 const *image2, float **delta1, float **delta2, const int& radius = 2);

private:
	jhw_gl::MyGLRenderer	m_gl_matching;
	jhw_gl::GLTex2d			m_gl_matching_tex1;
	jhw_gl::GLTex2d			m_gl_matching_tex2;

	jhw_gl::MyGLRenderer	m_gl_triangulation;
	jhw_gl::GLTex2d			m_gl_triangulation_tex1;
	jhw_gl::GLTex2d			m_gl_triangulation_tex2;

};

} // namespace mf
