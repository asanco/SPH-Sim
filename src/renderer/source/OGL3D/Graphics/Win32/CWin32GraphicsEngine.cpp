#include <OGL3D/Graphics/OGraphicsEngine.h>
#include <glad/glad_wgl.h>
#include <glad/glad.h>
#include <assert.h>
#include <OGL3D/Window/OWindow.h>
#include <stdexcept>

OGraphicsEngine::OGraphicsEngine()
{
	WNDCLASSEX windowClass = {};
	windowClass.style = CS_OWNDC;
	windowClass.lpfnWndProc = DefWindowProcA;
	windowClass.lpszClassName = "OGL3DDummyWindow";
	windowClass.cbSize = sizeof(WNDCLASSEX);

	auto classId = RegisterClassEx(&windowClass);

	HWND dummyWindow = CreateWindowEx(
		0,
		MAKEINTATOM(classId),
		"OGL3DDummyWindow",
		0,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		0,
		0,
		windowClass.hInstance,
		0);

	assert(dummyWindow);

	HDC dummyDC = GetDC(dummyWindow);

	PIXELFORMATDESCRIPTOR pfd = {};
	pfd.nSize = sizeof(pfd);
	pfd.nVersion = 1;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.cColorBits = 32;
	pfd.cAlphaBits = 8;
	pfd.iLayerType = PFD_MAIN_PLANE;
	pfd.cDepthBits = 24;
	pfd.cStencilBits = 8;

	int pixelFormat = ChoosePixelFormat(dummyDC, &pfd);
	SetPixelFormat(dummyDC, pixelFormat, &pfd);

	HGLRC dummyContext = wglCreateContext(dummyDC);
	assert(dummyContext);

	bool res = wglMakeCurrent(dummyDC, dummyContext);
	assert(res);


	if (!gladLoadWGL(dummyDC))
		OGL3D_ERROR("OGraphicsEngine - gladLoadWGL failed");

	if (!gladLoadGL())
		OGL3D_ERROR("OGraphicsEngine - gladLoadGL failed");


	wglMakeCurrent(dummyDC, 0);
	wglDeleteContext(dummyContext);
	ReleaseDC(dummyWindow, dummyDC);
	DestroyWindow(dummyWindow);
}


OGraphicsEngine::~OGraphicsEngine()
{
}