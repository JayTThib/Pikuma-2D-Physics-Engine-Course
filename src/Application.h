#ifndef APPLICATION_H
#define APPLICATION_H

#include "./Graphics.h"
#include "./Physics/Body.h"
#include <vector>

class Application {
    private:
        bool running = false;
        std::vector<Body*> bodies;
        Vec2 pushForce = Vec2(0.0f, 0.0f);

    public:
        Application() = default;
        ~Application() = default;
        bool IsRunning();
        void Setup();
        void Input();
        void Update();
        void Render();
        void Destroy();
};

#endif