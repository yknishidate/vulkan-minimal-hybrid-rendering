#include <vector>
#include <GLFW/glfw3.h>

struct Window
{
    static void init(int _width, int _height)
    {
        width = _width;
        height = _height;
        glfwInit();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        window = glfwCreateWindow(width, height, "Vulkan", nullptr, nullptr);
    }

    static bool shouldClose()
    {
        return glfwWindowShouldClose(window);
    }

    static void pollEvents()
    {
        glfwPollEvents();
    }

    static void terminate()
    {
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    static std::vector<const char*> getRequiredInstanceExtensions()
    {
        uint32_t extensionCount = 0;
        const char** extensions = glfwGetRequiredInstanceExtensions(&extensionCount);
        return { extensions, extensions + extensionCount };
    }

    static VkSurfaceKHR createSurface(VkInstance instance)
    {
        VkSurfaceKHR surface;
        if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
            throw std::runtime_error("failed to create window surface!");
        }
        return surface;
    }

    static int getWidth()
    {
        return width;
    }

    static int getHeight()
    {
        return height;
    }

    static inline GLFWwindow* window;
    static inline int width;
    static inline int height;
};
