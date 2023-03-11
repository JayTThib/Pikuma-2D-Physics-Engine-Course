https://user-images.githubusercontent.com/60585461/224510990-59d3f8ab-e418-420f-bd9f-f4de73229ffd.mp4

A 2D physics engine with SDL to handle rendering and input.

Created by following the course at: https://pikuma.com/courses/game-physics-engine-programming

Possible areas to improve-
- Better documentation.
- Support for concave polygons (currently only circles and convex polygons work).
- General stability. A stack of boxes still very slowly becomes more and more unstable.
- Playing around with the bias factor (so that something like a bridge made of circle joints is more stable).
- Contact caching / manifold caching. This will help with jittering as well.
- Broad phase for collisions, and a narrow phase.
- Support for continuous collision detection. Without this, very fast objects can pass through each other.

Other resources-
- Box2D Lite https://github.com/erincatto/box2d-lite
- Physics for Game Developers by Bryan Bywalec and David M. Bourg
- Game Physics by David H. Eberly
- 3D Math Primer for Graphics and Game Development by Fletcher Dunn and Ian Parbery
