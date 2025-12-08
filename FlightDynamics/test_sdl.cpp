#include <SDL3/SDL.h>
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << "Starting SDL test..." << std::endl;
    
    // Try to get version info first
    std::cout << "Attempting SDL_Init..." << std::endl;
    bool result = SDL_Init(0);
    
    std::cout << "SDL_Init result: " << (result ? "true" : "false") << std::endl;
    
    if (!result) {
        const char* error = SDL_GetError();
        if (error && error[0] != '\0') {
            std::cout << "SDL Error: " << error << std::endl;
        } else {
            std::cout << "SDL_Init failed but no error message" << std::endl;
        }
        return -1;
    }
    
    std::cout << "SDL initialized successfully!" << std::endl;
    SDL_Quit();
    return 0;
}
