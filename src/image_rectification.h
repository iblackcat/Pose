#ifndef IMAGE_RECTIFICATION_H
#define IMAGE_RECTIFICATION_H

#include "utils.h"
#include "gl_helper.h"

namespace mf {

class ImageRectification {
public:
	ImageRectification() {}
	~ImageRectification() {}

	bool init();
	bool get_rectified_pose(const CameraPose &p1, const CameraPose &p2, CameraPose &p1_rec, CameraPose &p2_rec);
	void image_rectification(const u32 *image1, const u32 *image2, const CameraPose &p1, const CameraPose &p2, u32 **image1_rec, u32 **image2_rec, CameraPose &p1_rec, CameraPose &p2_rec);

	bool destroy();

private:
	jhw_gl::MyGLRenderer	m_gl_rectify;
	jhw_gl::GLTex2d			m_gl_tex;
};

} //namespace mf


#endif //IMAGE_RECTIFICATION_H
