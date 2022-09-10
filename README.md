# elastic-collision-unity

Copyright (c) 2019 Yoann Berenguer

This code is not mine, I just took it from:
https://github.com/yoyoberenguer/Elastic-Collision/blob/version-1.0.2/ElasticCollision/Source/elastic_collision.c
and change it to make it usable on Unity. The reason I am uploading this to GitHub is just to eliminate the work
needed to be done to convert the above code to C#.

To use this script you just need to copy and paste this to your project, include this script to the script that you
need to use Elastic Collision and call momentum_angle_free1 method. 

momentum_angle_free1 method takes to inputs: ColliderObject which is a struct that needed 3 arguments. First one is velocity of
object before collision, second one is the mass of the object and the third one is position of the object. And this method returns
CollisionVectors struct which have velocities of two object after collision.

To understand how this script works, I strongly advise you to go to the original repo.
