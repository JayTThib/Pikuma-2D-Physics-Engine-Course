#include "Application.h"
#include "./Physics/Constants.h"
#include "./Physics/Force.h"
#include "./Physics/CollisionDetection.h"
#include "./Physics/Contact.h"
#include "./Physics/Constraint.h"
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
                Body* circ = new Body(CircleShape(50), x, y, 1.0f);
                circ->elasticity = 0.7f;
                //circ->friction = 0.7f;
                world->AddBody(circ);
                /*
                int randNum = 1 + (rand() % 3);
                if (randNum == 1) {
                    Body* circ = new Body(CircleShape(50), x, y, 1.0f);
                    circ->elasticity = 0.7f;
                    //circ->friction = 0.7f;
                    world->AddBody(circ);
                }
                else if (randNum == 2) {
                    Body* box = new Body(BoxShape(40, 40), x, y, 1.0f);
                    box->elasticity = 0.2f;
                    //box->friction = 0.7f;
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
                */
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

    for (Constraint* joint : world->GetConstraints()) {
        const Vec2 pa = joint->bodyA->LocalSpaceToWorldSpace(joint->pointA);
        const Vec2 pb = joint->bodyB->LocalSpaceToWorldSpace(joint->pointA);
        Graphics::DrawLine(pa.x, pa.y, pb.x, pb.y, 0xFF555555);
    }

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
    /*
    Body* bodyA = new Body(CircleShape(30), Graphics::Width() / 2.0f, Graphics::Height() / 2.0f, 0.0f);
    Body* bodyB = new Body(CircleShape(20), bodyA->position.x - 100, bodyA->position.y, 1.0f);
    world->AddBody(bodyA);
    world->AddBody(bodyB);

    JointConstraint* joint = new JointConstraint(bodyA, bodyB, bodyA->position);
    world->AddConstraint(joint);
    */



    
    const int NUM_BODIES = 8;
    for (int i = 0; i < NUM_BODIES; i++) {
        float mass = (i == 0) ? 0.0f : 1.0f;
        Body* body = new Body(BoxShape(30, 30), Graphics::Width() / 2.0 - (i * 40), 300, mass);
        world->AddBody(body);
    }

    // Add joints to connect them (distance constraints)
    for (int i = 0; i < NUM_BODIES - 1; i++) {
        Body* a = world->GetBodies()[i];
        Body* b = world->GetBodies()[i + 1];
        JointConstraint* joint = new JointConstraint(a, b, a->position);
        world->AddConstraint(joint);
    }
    /*
    Body* floor = new Body(BoxShape(Graphics::Width(), 20), Graphics::Width() / 2.0f, Graphics::Height(), 0.0f);
    world->AddBody(floor);
    Body* leftWall = new Body(BoxShape(20, Graphics::Height()), 0, Graphics::Height() / 2.0f, 0.0f);
    world->AddBody(leftWall);
    Body* rightWall = new Body(BoxShape(20, Graphics::Height()), Graphics::Width(), Graphics::Height() / 2.0f, 0.0f);
    world->AddBody(rightWall);
    */

    // Add a static circle in the middle of the screen
    //Body* bigBall = new Body(CircleShape(64), Graphics::Width() / 2.0, Graphics::Height() / 2.0, 0.0);
    //world->AddBody(bigBall);

    //Add a floor and walls to contain objects objects
    Body* floor = new Body(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2.0, Graphics::Height() - 50, 0.0);
    Body* leftWall = new Body(BoxShape(50, Graphics::Height() - 100), 50, Graphics::Height() / 2.0 - 25, 0.0);
    Body* rightWall = new Body(BoxShape(50, Graphics::Height() - 100), Graphics::Width() - 50, Graphics::Height() / 2.0 - 25, 0.0);
    floor->elasticity = 0.7;
    leftWall->elasticity = 0.2;
    rightWall->elasticity = 0.2;
    world->AddBody(floor);
    world->AddBody(leftWall);
    world->AddBody(rightWall);
}