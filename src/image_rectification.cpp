#include "utils.h"
#include "image_rectification.h"

namespace mf {

bool ImageRectification::init() {
	if (!m_gl_rectify.init(G.w, G.h))
		return false;
	if (!m_gl_rectify.setShaderFile("shader/default.vert", "shader/image_rectification.frag"))
		return false;
	m_gl_rectify.CreateVertexBuffer();
	m_gl_tex = m_gl_rectify.CreateTexture(&m_gl_tex.tex_id, G.w, G.h);
	return true;
} 

void ImageRectification::get_rectified_pose(const CameraPose &p1, const CameraPose &p2, CameraPose &p1_rec, CameraPose &p2_rec) {
	Eigen::Matrix3d A = (p1.intrinsics + p2.intrinsics) / 2;
	Eigen::Vector3d v1 = p1.center - p2.center;
	Eigen::Vector3d v2 = p1.R.row(2).cross(v1);
	Eigen::Vector3d v3 = v1.cross(v2);
	Eigen::Matrix3d R;
	R.row(0) = v1.normalized();
	R.row(1) = v2.normalized();
	R.row(2) = v3.normalized();

	p1_rec.R = R;
	p1_rec.intrinsics = A;
	p1_rec.t = -(R * p1.center);
	p1_rec.refreshByARt();

	p2_rec.R = R;
	p2_rec.intrinsics = A;
	p2_rec.t = -(R * p2.center);
	p2_rec.refreshByARt();
}

void ImageRectification::image_rectification(const u32 *image1, const u32 *image2, const CameraPose &p1, const CameraPose &p2, u32 **image1_rec, u32 **image2_rec) {
	CameraPose p1_rec, p2_rec;
	
	get_rectified_pose(p1, p2, p1_rec, p2_rec);

	Eigen::Matrix3d rect_tran1 = p1.Q * p1_rec.Q.inverse();
	Eigen::Matrix3d rect_tran2 = p2.Q * p2_rec.Q.inverse();

	m_gl_rectify.useRenderer();

	if (!image1_rec) { free(image1_rec); image1_rec = NULL; }
	if (!image2_rec) { free(image2_rec); image2_rec = NULL; }

	float trans1[16] = { 0 }, trans2[16] = { 0 };
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			if (i == 3 && j == 3) { trans1[i * 4 + j] = 1.0f; trans2[i * 4 + j] = 1.0f; }
			else if (i == 3 || j == 3) { trans1[i * 4 + j] = 0.0f; trans2[i * 4 + j] = 0.0f; }
			else { trans1[i * 4 + j] = (float)rect_tran1(j, i); trans2[i * 4 + j] = (float)rect_tran2(j, i); }
		}
	}

	u32 *rec1 = nullptr, *rec2 = nullptr;

	m_gl_rectify.setUniform1("m_w", G.w);
	m_gl_rectify.setUniform1("m_h", G.h);
	m_gl_rectify.setUniform4v("Trans", trans1);
	m_gl_rectify.setTexSub2D("tex", m_gl_tex, 0, GL_TEXTURE0, image1);
	rec1 = m_gl_rectify.RenderScence<u32>();

	m_gl_rectify.setUniform1("m_w", G.w);
	m_gl_rectify.setUniform1("m_h", G.h);
	m_gl_rectify.setUniform4v("Trans", trans2);
	m_gl_rectify.setTexSub2D("tex", m_gl_tex, 0, GL_TEXTURE0, image2);
	rec2 = m_gl_rectify.RenderScence<u32>();

	*image1_rec = rec1;
	*image2_rec = rec2;
}

} //namespace mf
