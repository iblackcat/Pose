#version 450

in vec2 st;

uniform int			m_w;
uniform int			m_h;
uniform mat4		Trans;
uniform sampler2D	tex;

out vec4 FragColor;  
  
void main()  
{  
//	FragColor = vec4(1.0, 0.0, 0.0, 1.0);
//	return;

	float dx = 1.0 / m_w;
	float dy = 1.0 / m_h;
	vec4 m = Trans*vec4((st.x-dx/2.0)*m_w, ((st.y-dy/2.0)*m_h), 1.0, 0.0);
	vec2 haha = vec2((m[0]/m[2])/m_w+dx/2.0, ((m[1]/m[2]))/m_h+dy/2.0);
	vec4 C;
	if (haha.x < 0.0 || haha.y < 0.0 || haha.x > 1.0 || haha.y > 1.0) 
		C = vec4(0.0, 0.0, 0.0, 0.0);
	//else if (int(st.y*480) % 30 == 0) 
	//	C = vec4(0.6, 0.6, 0.1, 1.0);
	else {
		C = texture2D(tex, vec2(haha.x, haha.y));
	}

	FragColor = C;
}  