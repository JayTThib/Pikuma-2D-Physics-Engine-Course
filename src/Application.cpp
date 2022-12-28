#include "Application.h"
#include "./Physics/Constants.h"

bool Application::IsRunning() {
    return running;
}

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void Application::Setup() {
    running = Graphics::OpenWindow();

    Particle* smallBall = new Particle(50, 100, 1.0f);
    smallBall->radius = 4;
    particles.push_back(smallBall);

    Particle* bigBall = new Particle(50, 200, 3.0f);
    bigBall->radius = 12;
    particles.push_back(bigBall);
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void Application::Input() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                    running = false;
                break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Update() {
    //Wait some time until it reaches the target frame time in milliseconds.
    static int timePreviousFrame;
    int timeToWait = MILLISECONDS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
    if (timeToWait > 0) {
        SDL_Delay(timeToWait);
    }

    //Calc deltaTime in seconds
    float deltaTime = (SDL_GetTicks() - timePreviousFrame) / 1000.0f;

    if (deltaTime > 0.016f) {
        deltaTime = 0.016f;
    }

    //Set the time of the current frame to be used in the next one.
    timePreviousFrame = SDL_GetTicks();

    Vec2 wind = Vec2(0.2f * PIXELS_PER_METER, 0.0f);
    for (auto particle: particles) {
        particle->AddForce(wind);
    }

    Vec2 weight = Vec2(0.0f, 9.8f * PIXELS_PER_METER);
    for (auto particle : particles) {
        particle->AddForce(weight);
    }

    //Integrate the accel and velocity of the new position
    for (auto particle: particles) {
        particle->Integrate(deltaTime);
    }
    
    //collision here?
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Render() {
    Graphics::ClearScreen(0xFF056263);

    for (auto particle : particles) {
        Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    }
    
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void Application::Destroy() {
    for (auto particle: particles) {
        delete particle;
    }
    Graphics::CloseWindow();
}