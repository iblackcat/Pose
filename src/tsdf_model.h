#pragma once

#include "utils.h"
#include "gl_helper.h"

namespace mf {

class TSDFModel
{
public:
	TSDFModel();
	~TSDFModel();

	bool init(float size = 4.0);
	bool model_updating(u32* image, u8* depth, CameraPose pose);
	bool ray_tracing(const CameraPose &pose, u32 **image, u8 **depth, u8 **weight);

	bool destroy();

	u32 *get_modelC() { return m_modelC; }
	u32 *get_modelSW() { return m_modelSW; }
	u32 modelC(int i, int j) { return m_modelC[i*ModelTexSize + j]; }
	u32 modelSW(int i, int j) { return m_modelSW[i*ModelTexSize + j]; }

	static const int		ModelSize = 256;
	static const int		ModelTexSize = 4096;
private:
	const int				Mu = 4; // 8;

	float					m_size; // in millimeter
	u32*					m_modelC = nullptr;
	u32*					m_modelSW = nullptr;
	Eigen::Vector3d			m_axis[6];
	
	jhw_gl::MyGLRenderer	m_gl_updating;
	jhw_gl::GLTex2d			m_gl_updating_teximage;
	jhw_gl::GLTex2d			m_gl_updating_texdepth;
	jhw_gl::GLTex2d			m_gl_updating_texmodel;

	jhw_gl::MyGLRenderer	m_gl_raytracing;
	jhw_gl::GLTex2d			m_gl_raytracing_texmodelC;
	jhw_gl::GLTex2d			m_gl_raytracing_texmodelSW;

	jhw_gl::MyGLRenderer	m_gl_raytracing_single;
	jhw_gl::GLTex2d			m_gl_raytracing_single_texmodel;
};

} //namespace mf
