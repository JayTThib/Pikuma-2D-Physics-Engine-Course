#ifndef APPLICATION_H
#define APPLICATION_H

#include "./Graphics.h"
#include "./Physics/Body.h"
#include "./Physics/World.h"
#include <vector>

class Application {
    private:
        bool debug = false;
        bool running = false;
        World* world;
        unsigned randSeed;

    public:
        Application() = default;
        ~Application() = default;
        bool IsRunning();
        void Setup();
        void Input();
        void Update();
        void Render();
        void Destroy();

        void InitWorld();
};

#endif