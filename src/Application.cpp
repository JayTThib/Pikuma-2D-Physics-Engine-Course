#include "Application.h"
#include "./Physics/Constants.h"
#include "./Physics/Force.h"
#include "./Physics/CollisionDetection.h"
#include "./Physics/Contact.h"

bool Application::IsRunning() {
    return running;
}

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void Application::Setup() {
    running = Graphics::OpenWindow();

    const float wallFric = 0.4f;

    Body* floor = new Body(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2.0f, Graphics::Height() - 50, 0.0f);
    floor->elasticity = 0.5f;
    floor->friction = wallFric;
    bodies.push_back(floor);

    Body* leftWall = new Body(BoxShape(50, 300), 0, Graphics::Height() - 200, 0.0f);
    leftWall->elasticity = 0.5f;
    leftWall->friction = wallFric;
    bodies.push_back(leftWall);

    Body* rightWall = new Body(BoxShape(50, 300), Graphics::Width(), Graphics::Height() - 200, 0.0f);
    rightWall->elasticity = 0.5f;
    rightWall->friction = wallFric;
    bodies.push_back(rightWall);

    Body* bigBox = new Body(BoxShape(200, 200), Graphics::Width() / 2.0f, Graphics::Height() / 2.0f, 0.0f);
    bigBox->rotation = 1.4f;
    bigBox->elasticity = 0.7f;
    bigBox->friction = wallFric;
    bodies.push_back(bigBox);
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

            /*
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_UP) {
                    pushForce.y = -50 * PIXELS_PER_METER;
                }
                if (event.key.keysym.sym == SDLK_RIGHT) {
                    pushForce.x = 50 * PIXELS_PER_METER;
                }
                if (event.key.keysym.sym == SDLK_DOWN) {
                    pushForce.y = 50 * PIXELS_PER_METER;
                }
                if (event.key.keysym.sym == SDLK_LEFT) {
                    pushForce.x = -50 * PIXELS_PER_METER;
                }
                break;

            case SDL_KEYUP:
                if (event.key.keysym.sym == SDLK_UP) {
                    pushForce.y = 0;
                }
                if (event.key.keysym.sym == SDLK_RIGHT) {
                    pushForce.x = 0;
                }
                if (event.key.keysym.sym == SDLK_DOWN) {
                    pushForce.y = 0;
                }
                if (event.key.keysym.sym == SDLK_LEFT) {
                    pushForce.x = 0;
                }
                break;
            */

            case SDL_MOUSEMOTION:
                /*
                int x, y;
                SDL_GetMouseState(&x, &y);
                bodies[0]->position.x = x;
                bodies[0]->position.y = y;
                */
                break;

            case SDL_MOUSEBUTTONDOWN:
                int x, y;
                SDL_GetMouseState(&x, &y);
                Body* circ = new Body(CircleShape(30), x, y, 1.0f);
                circ->elasticity = 0.5f;
                circ->friction = 0.4f;
                bodies.push_back(circ);
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

    //Apply forces
    for (Body* body : bodies) {
        Vec2 weight = Vec2(0.0f, body->mass * 9.8f * PIXELS_PER_METER);
        body->AddForce(weight);
        //body->AddForce(pushForce);
        //float torque = 800;
        //body->AddTorque(torque);
    }

    //Integrate the accel and velocity of the new position
    for (Body* body : bodies) {
        body->Update(deltaTime);
    }
    
    //Reset collision flag for all bodies
    for (Body* &body : bodies) {
        body->isColliding = false;
    }

    //Check all rigidbodies with the other rigidbodies for collision
    for (int i = 0; i <= bodies.size() - 1; i++) {
        for (int j = i + 1; j < bodies.size(); j++) {
            Body* bodyA = bodies[i];
            Body* bodyB = bodies[j];
            Contact contact;
            if (CollisionDetection::IsColliding(bodyA, bodyB, contact)) {
                contact.ResolveCollision();//Resolve using impulse method

                //Draw debug info
                Graphics::DrawFillCircle(contact.start.x, contact.start.y, 3, 0xFFFF00FF);
                Graphics::DrawFillCircle(contact.end.x, contact.end.y, 3, 0xFFFF00FF);
                Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15, contact.start.y + contact.normal.y  * 15, 0xFFFF00FF);

                bodyA->isColliding = true;
                bodyB->isColliding = true;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Render() {
    
    for (Body* body : bodies) {
        Uint32 color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;
        switch (body->shape->GetType()) {
            case BOX: {
                BoxShape* boxShape = (BoxShape*)body->shape;
                Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, color);
                }
                break;

            case CIRCLE: {
                CircleShape* circleShape = (CircleShape*)body->shape;
                Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, color);
                }
                break;

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
    for (Body* body : bodies) {
        delete body;
    }
    Graphics::CloseWindow();
}