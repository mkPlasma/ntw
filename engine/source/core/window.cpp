#include"window.h"


Window::Window(Options& options) : options_(options), window_(nullptr), mouseLocked_(false) {
	
}

Window::~Window(){
	glfwDestroyWindow(window_);
	glfwTerminate();
}

void Window::init(){

	centerX_ = options_.graphics.resolutionX / 2;
	centerY_ = options_.graphics.resolutionY / 2;

	int numKeys = NTW_KEYS_SIZE;

#ifdef __NTW_DEBUG__
	numKeys = NTW_KEYS_DEBUG_SIZE;
#endif

	for(int i = 0; i < numKeys; i++)
		keys_[i] = false;

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

	window_ = glfwCreateWindow(options_.graphics.resolutionX, options_.graphics.resolutionY, "NTW", NULL, NULL);

	if(!window_){
		glfwTerminate();
		ntw::fatalError("Failed to create GLFW window");
	}

	// Primary monitor
	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);

	// Fullscreen
	if(options_.graphics.fullscreen)
		glfwSetWindowMonitor(window_, monitor, 0, 0, options_.graphics.resolutionX, options_.graphics.resolutionY, mode->refreshRate);

	// Windowed mode, center window
	else
		glfwSetWindowPos(window_, (mode->width - options_.graphics.resolutionX) / 2, (mode->height - options_.graphics.resolutionY) / 2);


	glfwMakeContextCurrent(window_);

	if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
		glfwTerminate();
		ntw::fatalError("Failed to initialize GLAD");
	}

	glViewport(0, 0, options_.graphics.resolutionX, options_.graphics.resolutionY);

	// VSync
	glfwSwapInterval(options_.graphics.useVSync);
}

void Window::setMouseLock(bool lock){
	mouseLocked_ = lock;
	glfwSetInputMode(window_, GLFW_CURSOR, lock ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}

void Window::centerMousePosition(){
	if(mouseLocked_)
		glfwSetCursorPos(window_, centerX_, centerY_);
}

void Window::updateKeys(){

	int numKeys = NTW_KEYS_SIZE;

#ifdef __NTW_DEBUG__
	numKeys = NTW_KEYS_DEBUG_SIZE;
#endif

	// Update each key status in array
	for(int i = 0; i < numKeys; i++){
		bool prev = keys_[i];
		keys_[i] = glfwGetKey(window_, options_.control.keys[i]) == GLFW_PRESS;

		// key in keyPress array is true only when pressed initially
		keyPress_[i] = !prev && keys_[i];
	}
}

bool Window::isKeyDown(int key){
	return keys_[key];
}

bool Window::isKeyPressed(int key){
	return keyPress_[key];
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
