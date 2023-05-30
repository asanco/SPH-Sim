#include <OGL3D/Renderer/ORenderer3D.h>
#include <OGL3D/Window/OWindow.h>
#include <Windows.h>


void ORenderer3D::run()
{
	onCreate();
	while (m_isRunning)
	{
		MSG msg = {};
		if (PeekMessage(&msg, HWND(), NULL, NULL, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
			{
				m_isRunning = false;
				continue;
			}
			else
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}

		onUpdateInternal();
	}

	onQuit();
}