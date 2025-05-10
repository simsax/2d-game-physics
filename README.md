# 2d-game-physics

A 2d rigid body physics engine written in C.

This is my own C version of the physics engine from the [Pikuma game physics course](https://pikuma.com/courses/game-physics-engine-programming).
The main differences, besides the language, are the math calculation of the constraints and the addition of contact caching for the penetration constraints in order to improve stability.

The math equations for the constraints have been modified to remove heap allocations; this was achieved by simply pre-computing the Jacobian calculations and removing all the matrix multiplications from the code.

Contact caching is implemented using a hash table for the manifolds (a manifold is just a fancy name for the collection of penetration constraints between 2 bodies).

Graphics is done with raylib.

## How to build & run

I only tested this on Linux with gcc, if you have a similar setup you can just:

```
make rel
make run
```

If you manage to get it running, you can see that there are 4 demos to select. In each demo, you can spawn circles and squares by left/right click of the mouse.

## Future work 

I had more ambitious goals for this one but I ended up getting sidetracked, so I'll just leave it like this for now. 
After all, the main goal was to learn how physics works in videogames by implementing it from scratch, and I can say that goal has been achieved.
Some features and optimizations that I'd have liked to add are:

* broad phase collision detection
* sleeping islands
* continuous collision detection

But I guess I'll leave them for a future project...

## Random showcase

https://github.com/user-attachments/assets/50ae29ea-ed1b-43d2-99ae-b09b8a1eae69

https://github.com/user-attachments/assets/396ec79e-5bf4-46d8-bc2e-eb6d273b633d


## References

* [Pikuma's game physics course](https://pikuma.com/courses/game-physics-engine-programming)
* [Box2D lite](https://github.com/erincatto/box2d-lite) 
* [Raylib](https://www.raylib.com/)
* http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/ 
* https://www.youtube.com/watch?v=IjObMkufU9I (In Demo 4 I tried to replicate this)
