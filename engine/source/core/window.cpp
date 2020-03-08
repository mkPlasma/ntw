#include"window.h"


Window::Window(GraphicsOptions& gOptions) : gOptions_(gOptions), window_(nullptr), mouseLocked_(false) {
	
}

Window::~Window(){
	glfwDestroyWindow(window_);
	glfwTerminate();
}

void Window::init(){

	centerX_ = gOptions_.resolutionX / 2;
	centerY_ = gOptions_.resolutionY / 2;

	// Initialize GLFW and check success
	if(!glfwInit())
		exit(EXIT_FAILURE);


	// Require OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Disable manual resizing
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

	// Use raw mouse input
	if(glfwRawMouseMotionSupported())
		glfwWindowHint(GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);


	window_ = glfwCreateWindow(gOptions_.resolutionX, gOptions_.resolutionY, "NTW", NULL, NULL);

	if(!window_){
		glfwTerminate();
		ntw::fatalError("Failed to create GLFW window");
	}

	// Center window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	glfwSetWindowPos(window_, (mode->width - gOptions_.resolutionX) / 2, (mode->height - gOptions_.resolutionY) / 2);


	glfwMakeContextCurrent(window_);

	if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
		glfwTerminate();
		ntw::fatalError("Failed to initialize GLAD");
	}

	glViewport(0, 0, gOptions_.resolutionX, gOptions_.resolutionY);

	// VSync
	glfwSwapInterval(gOptions_.useVSync);
}

void Window::setMouseLock(bool lock){
	mouseLocked_ = lock;
	glfwSetInputMode(window_, GLFW_CURSOR, lock ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}

void Window::centerMousePosition(){
	if(mouseLocked_)
		glfwSetCursorPos(window_, centerX_, centerY_);
}

bool Window::isKeyDown(int key){
	return glfwGetKey(window_, key) == GLFW_PRESS;
}

bool Window::isMouseButtonDown(int button){
	return glfwGetMouseButton(window_, button) == GLFW_PRESS;
}

int Window::getMouseX(){
	double x;
	glfwGetCursorPos(window_, &x, nullptr);
	return (int)(mouseLocked_ ? x - centerX_ : x);
}

int Window::getMouseY(){
	double y;
	glfwGetCursorPos(window_, nullptr, &y);
	return (int)(mouseLocked_ ? y - centerY_ : y);
}

bool Window::isMouseInWindow(){
	return glfwGetWindowAttrib(window_, GLFW_HOVERED);
}

bool Window::isMouseLocked(){
	return mouseLocked_;
}

GLFWwindow* Window::getWinPtr(){
	return window_;
}
