#include "Application.h"
#include "./Physics/Constants.h"
#include "./Physics/Force.h"
#include "./Physics/CollisionDetection.h"
#include "./Physics/Contact.h"
#include <iostream>

bool Application::IsRunning() {
    return running;
}

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void Application::Setup() {
    running = Graphics::OpenWindow();

    randSeed = time(0);
    srand(randSeed);

    InitWorld();
    GenerateTerrain();
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
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_d) {
                    debug = !debug;
                }
                if (event.key.keysym.sym == SDLK_t) {
                    /*
                    for (Body* body : bodies) {
                        delete body;
                    }
                    bodies.clear();
                    */
                    //GenerateTerrain();
                }
                break;

            case SDL_MOUSEBUTTONDOWN:
                int x, y;
                SDL_GetMouseState(&x, &y);
                
                int randNum = 1 + (rand() % 3);
                if (randNum == 1) {
                    Body* circ = new Body(CircleShape(50), x, y, 1.0f);
                    circ->elasticity = 0.5f;
                    circ->friction = 0.7f;
                    world->AddBody(circ);
                }
                else if (randNum == 2) {
                    Body* box = new Body(BoxShape(40, 40), x, y, 1.0f);
                    box->elasticity = 0.5f;
                    box->friction = 0.7f;
                    world->AddBody(box);
                }
                else {
                    std::vector<Vec2> polyVertices{
                    Vec2(20, 60),
                    Vec2(-40, 20),
                    Vec2(-20, -60),
                    Vec2(20, -60),
                    Vec2(40, 20)
                    };
                    Body* poly = new Body(PolygonShape(polyVertices), x, y, 2.0f);
                    poly->elasticity = 0.5f;
                    poly->friction = 0.7f;
                    world->AddBody(poly);
                }
                break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Update() {
    Graphics::ClearScreen(0x00000000);

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

    world->Update(deltaTime);
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Render() {
    for (Body* body : world->GetBodies()) {
        Uint32 color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;
        switch (body->shape->GetType()) {
            case BOX: {
                BoxShape* boxShape = (BoxShape*)body->shape;
                Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, color);
            } break;

            case CIRCLE: {
                CircleShape* circleShape = (CircleShape*)body->shape;
                Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, color);
            } break;

            case POLYGON: {
                PolygonShape* polyShape = (PolygonShape*)body->shape;
                Graphics::DrawPolygon(body->position.x, body->position.y, polyShape->worldVertices, color);
            } break;

            default:
                break;
        }
    }

    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void Application::Destroy() {
    delete world;
    Graphics::CloseWindow();
}

void Application::InitWorld() {
    world = new World(-9.8f);
}

void Application::GenerateTerrain() {
    const float wallFric = 0.4f;

    Body* floor = new Body(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2.0f, Graphics::Height() - 50, 0.0f);
    floor->elasticity = 0.5f;
    floor->friction = wallFric;
    world->AddBody(floor);

    Body* leftWall = new Body(BoxShape(50, 300), 0, Graphics::Height() - 200, 0.0f);
    leftWall->elasticity = 0.5f;
    leftWall->friction = wallFric;
    world->AddBody(leftWall);

    Body* rightWall = new Body(BoxShape(50, 300), Graphics::Width(), Graphics::Height() - 200, 0.0f);
    rightWall->elasticity = 0.5f;
    rightWall->friction = wallFric;
    world->AddBody(rightWall);

    Body* bigBox = new Body(BoxShape(200, 200), Graphics::Width() / 2.0f, Graphics::Height() / 2.0f, 0.0f);
    bigBox->rotation = 1.4f;
    bigBox->elasticity = 0.7f;
    bigBox->friction = wallFric;
    world->AddBody(bigBox);
}