#include "tsdf_model.h"

namespace mf {

TSDFModel::TSDFModel()
{
}


TSDFModel::~TSDFModel()
{
}

bool TSDFModel::init(float size) {
	m_size = size;
	//model updating renderer
	if (!m_gl_updating.init(ModelTexSize, ModelTexSize))
		return false;
	if (!m_gl_updating.setShaderFile("shader/default.vert", "shader/model_updating.frag"))
		return false;
	m_gl_updating.CreateVertexBuffer();
	m_gl_updating_teximage = m_gl_updating.CreateTexture(&m_gl_updating_teximage.tex_id, G.w, G.h);
	m_gl_updating_texdepth = m_gl_updating.CreateTexture(&m_gl_updating_texdepth.tex_id, G.w, G.h, GL_R8, GL_RED, GL_UNSIGNED_BYTE);
	m_gl_updating_texmodel = m_gl_updating.CreateTexture(&m_gl_updating_texmodel.tex_id, ModelTexSize, ModelTexSize);

	
	//ray tracing renderer
	if (!m_gl_raytracing.init(G.w, G.h))
		return false;
	if (!m_gl_raytracing.setShaderFile("shader/default.vert", "shader/ray_tracing.frag"))
		return false;
	m_gl_raytracing.CreateVertexBuffer();
	m_gl_raytracing_texmodelC = m_gl_raytracing.CreateTexture(&m_gl_raytracing_texmodelC.tex_id, ModelTexSize, ModelTexSize);
	m_gl_raytracing_texmodelSW = m_gl_raytracing.CreateTexture(&m_gl_raytracing_texmodelSW.tex_id, ModelTexSize, ModelTexSize);
	
	/*
	//ray tracing (single) renderer
	if (!m_gl_raytracing_single.init(G.w, G.h, GL_R8, GL_RED, GL_UNSIGNED_BYTE))
		return false;
	if (!m_gl_raytracing_single.setShaderFile("shader/default.vert", "shader/ray_tracing_single.frag"))
		return false;
	m_gl_raytracing_single.CreateVertexBuffer();
	m_gl_raytracing_single_texmodel = m_gl_raytracing_single.CreateTexture(&m_gl_raytracing_single_texmodel.tex_id, ModelTexSize, ModelTexSize);
	*/

	//initiate model
	if (m_modelC) {
		free(m_modelC);
		m_modelC = nullptr;
	}
	m_modelC = (u32*)malloc(sizeof(u32) * ModelTexSize * ModelTexSize);

	if (m_modelSW) {
		free(m_modelSW);
		m_modelSW = NULL;
	}
	m_modelSW = (u32*)malloc(sizeof(u32) * ModelTexSize * ModelTexSize);

	memset(m_modelC, 0, sizeof(u32) * ModelTexSize * ModelTexSize);
	for (int i = 0; i < ModelTexSize; ++i) {
		for (int j = 0; j < ModelTexSize; ++j) {
			m_modelSW[i*ModelTexSize + j] = u32(Mu + 128);
		}
	}

	m_axis[0] = Eigen::Vector3d( 1.0,  0.0,  0.0);
	m_axis[1] = Eigen::Vector3d(-1.0,  0.0,  0.0);
	m_axis[2] = Eigen::Vector3d( 0.0,  1.0,  0.0);
	m_axis[3] = Eigen::Vector3d( 0.0, -1.0,  0.0);
	m_axis[4] = Eigen::Vector3d( 0.0,  0.0,  1.0);
	m_axis[5] = Eigen::Vector3d( 0.0,  0.0, -1.0);

	return true;
}

bool TSDFModel::destroy() {
	m_gl_updating.destroy();
	m_gl_raytracing.destroy();
	m_gl_raytracing_single.destroy();

	return true;
}

bool TSDFModel::model_updating(u32* image, u8* depth, CameraPose pose) {
	if (!image || !depth) return false;
	if (!m_modelC || !m_modelSW) return false;

	m_gl_updating.useRenderer();

	float Q[16];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Q[j * 4 + i] = static_cast<float>(pose.Q(i, j));
		}
		Q[3 * 4 + i] = static_cast<float>(pose.q[i]);
		Q[i * 4 + 3] = 0.0f;
	}
	Q[3 * 4 + 3] = 1.0;

	//C
	m_gl_updating.setUniform1("m_w", G.w);
	m_gl_updating.setUniform1("m_h", G.h);
	m_gl_updating.setUniform1("ModelSize", static_cast<int>(ModelSize));
	m_gl_updating.setUniform1("ModelTexSize", static_cast<int>(ModelTexSize));
	m_gl_updating.setUniform1("Mu", static_cast<int>(Mu));
	m_gl_updating.setUniform1("size", m_size);
	m_gl_updating.setUniform1("isC_flag", static_cast<int>(1));
	m_gl_updating.setUniform4v("Q", Q);
	m_gl_updating.setTexSub2D("tex_image", m_gl_updating_teximage, 0, GL_TEXTURE0, image);
	m_gl_updating.setTexSub2D("tex_depth", m_gl_updating_texdepth, 1, GL_TEXTURE1, depth);
	m_gl_updating.setTexSub2D("model", m_gl_updating_texmodel, 2, GL_TEXTURE2, m_modelC);
	free(m_modelC); m_modelC = NULL;
	m_modelC = m_gl_updating.RenderScence<u32>();

	//SW
	m_gl_updating.setUniform1("m_w", G.w);
	m_gl_updating.setUniform1("m_h", G.h);
	m_gl_updating.setUniform1("ModelSize", static_cast<int>(ModelSize));
	m_gl_updating.setUniform1("ModelTexSize", static_cast<int>(ModelTexSize));
	m_gl_updating.setUniform1("Mu", static_cast<int>(Mu));
	m_gl_updating.setUniform1("size", m_size);
	m_gl_updating.setUniform1("isC_flag", static_cast<int>(0));
	m_gl_updating.setUniform4v("Q", Q);
	m_gl_updating.setTexSub2D("tex_image", m_gl_updating_teximage, 0, GL_TEXTURE0, image);
	m_gl_updating.setTexSub2D("tex_depth", m_gl_updating_texdepth, 1, GL_TEXTURE1, depth);
	m_gl_updating.setTexSub2D("model", m_gl_updating_texmodel, 2, GL_TEXTURE2, m_modelSW);
	free(m_modelSW); m_modelSW = NULL;
	m_modelSW = m_gl_updating.RenderScence<u32>();

	return true;
}

bool TSDFModel::ray_tracing(const CameraPose &pose, u32 **image, u8 **depth, u8 **weight) {
	u32 *I = nullptr, *D = nullptr, *W = nullptr;
	double tmp = 0.0;
	int axis_tmp = 0;
	for (int i = 0; i < 6; ++i) {
		if (pose.R.col(2).dot(m_axis[i])> tmp) {
			tmp = pose.R.col(2).dot(m_axis[i]);
			axis_tmp = i;
		}
	}

	Eigen::Matrix3d invQ = pose.Q.inverse();
	float iQ[9] = { 0.0 };
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			iQ[i * 3 + j] = invQ(i, j);
		}
	}

	m_gl_raytracing.useRenderer();
	m_gl_raytracing.setUniform1("m_w", G.w);
	m_gl_raytracing.setUniform1("m_h", G.h);
	m_gl_raytracing.setUniform1("ModelSize", static_cast<int>(ModelSize));
	m_gl_raytracing.setUniform1("tex_size", static_cast<int>(ModelTexSize));
	m_gl_raytracing.setUniform1("size", m_size);
	m_gl_raytracing.setUniform1("flag", 0);
	m_gl_raytracing.setUniform1("Mu", static_cast<int>(Mu));
	m_gl_raytracing.setUniform1("Axis", axis_tmp);
	m_gl_raytracing.setUniform3v("invQ", iQ);
	m_gl_raytracing.setUniform3("q", static_cast<float>(pose.q[0]), static_cast<float>(pose.q[1]), static_cast<float>(pose.q[2]));
	m_gl_raytracing.setTexSub2D("modelC", m_gl_raytracing_texmodelC, 0, GL_TEXTURE0, m_modelC);
	m_gl_raytracing.setTexSub2D("modelSW", m_gl_raytracing_texmodelSW, 1, GL_TEXTURE1, m_modelSW);
	
	I = m_gl_raytracing.RenderScence<u32>();
	*image = I;


	return true;
}

} //namespace mf