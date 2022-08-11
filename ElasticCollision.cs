/*

Copyright (c) 2019 Yoann Berenguer

This code is not mine, I just took it from:
https://github.com/yoyoberenguer/Elastic-Collision/blob/version-1.0.2/ElasticCollision/Source/elastic_collision.c
and change it to make it usable on Unity. The reason I am uploading this to GitHub is just to eliminate the work
needed to be done to convert the above code to C#.

*/

using System;
using System.Numerics;
using UnityEngine;
using UnityEngine.Assertions;
using Vector2 = UnityEngine.Vector2;

namespace Elastic_Collision
{
	public struct ColliderObject
    {
        public Vector2 Vector;
        public float Mass;
        public Vector2 Centre;
    }

    public struct CollisionVectors
    {
        public Vector2 V12;
        public Vector2 V21;
    }
	
	
    public class ElasticCollision
    {
        private float contact_angle(Vector2 object1, Vector2 object2)
        {
            float phi =  (float)Mathf.Atan2(object2.y - object1.y, object2.x - object1.x);
            if (phi > 0.0) {
                phi = phi - 2.0f * Mathf.PI;
            } 
            // phi *= -1.0;
            return phi;
        }

        private float theta_angle(Vector2 vector)
        {
            float theta = 0.0f;
            float length = vector.magnitude;

            if (length == 0.0f)
                return -1;
            
            theta = (float)Mathf.Acos(vector.x / length);
            if (vector.y < 0.0)
            {
                theta *= -1.0f;
            }

            theta = (float)Mathf.Min(theta, Mathf.PI);
            theta = (float)Mathf.Max(theta, -Mathf.PI);
            return (float)theta;
        }
        
        // ----------------------------------------------------------------- TRIGONOMETRY METHOD ------------------------------------------------------------------------------------

        private Vector2 v12_vector_components(float v1, float v2, float theta1, float theta2, float phi,
            float m1, float m2)
        {
            float m12 = m1 + m2;
            float theta1_phi = theta1 - phi;
            
            Debug.Assert(v1 >= 0.0f && v2 >= 0.0f);
            Debug.Assert(m1 + m2 > 0.0f);

            Vector2 v12 = Vector2.zero;
            float numerator = v1 * Mathf.Cos(theta1_phi) * (m1 - m2) + (2.0f * m2 * v2) * Mathf.Cos(theta2 - phi);
            v12.x = numerator * Mathf.Cos(phi) / m12 + v1 * Mathf.Sin(theta1_phi) * Mathf.Cos(phi + Mathf.PI / 2);
            v12.y = numerator * Mathf.Sin(phi) / m12 + v1 * Mathf.Sin(theta1_phi) * Mathf.Sin(phi + Mathf.PI / 2);
            
            return v12;
        }

        private Vector2 v21_vector_components(float v1, float v2, float theta1, float theta2, float phi,
            float m1, float m2)
        {
            float inv_mass = 1.0f / (m1 + m2);
            float theta2_phi = theta2 - phi;
            
            Debug.Assert(v1 >= 0.0f && v2 >= 0.0f);
            Debug.Assert(m1 + m2 > 0.0f);

            Vector2 v21 = Vector2.zero;
            float numerator = v2 * Mathf.Cos(theta2_phi) * (m2 - m1) + (2.0f * m1 * v1) * Mathf.Cos(theta1 - phi);
            v21.x = numerator * Mathf.Cos(phi) * inv_mass + v2 * Mathf.Sin(theta2_phi) * Mathf.Cos(phi + Mathf.PI / 2);
            v21.y = numerator * Mathf.Sin(phi) * inv_mass + v2 * Mathf.Sin(theta2_phi) * Mathf.Sin(phi + Mathf.PI / 2);

            return v21;
        }

        public CollisionVectors momentum_t(ColliderObject obj1, ColliderObject obj2)
        {
            Vector2 v12, v21;
            CollisionVectors vec;
            float phi = contact_angle(obj1.Centre, obj2.Centre);
            float theta1 = theta_angle(obj1.Vector);
            float theta2 = theta_angle(obj2.Vector);
            float v1 = obj1.Vector.magnitude;
            float v2 = obj2.Vector.magnitude;
            v12 = v12_vector_components(v1, v2, theta1, theta2, phi, obj1.Mass, obj2.Mass);
            v21 = v21_vector_components(v1, v2, theta1, theta2, phi, obj1.Mass, obj2.Mass);
            vec.V12 = v12;
            vec.V21 = v21;

            return vec;
        }
        
        // ---------------------------------------------------------- ANGLE FREE METHOD -------------------------------------------------------------------------------

        private Vector2 v1_vector_components(Vector2 v1, Vector2 v2, float m1, float m2, Vector2 x1, Vector2 x2)
        {
            Debug.Assert(m1 + m2 > 0.0f);
            Debug.Assert(x1.x != x2.x && x1.y != x2.y);

            float mass = 2.0f * m2 / (m1 + m2);
            Vector2 v12, x12;

            v12 = Vector2.zero;
            x12 = Vector2.zero;
            v12 = v1 - v2;
            x12 = x1 - x2;

            float x12_length = x12.magnitude;
            float d = Vector2.Dot(v12, x12);

            float temp = mass * d / (x12_length * x12_length);
            x12.x = x12.x * temp;
            x12.y = x12.y * temp;

            return v1 - x12;
        }

        private Vector2 v2_vector_components(Vector2 v1, Vector2 v2, float m1, float m2, Vector2 x1, Vector2 x2)
        {
            Debug.Assert(m1 + m2 > 0.0);
            Debug.Assert(x1.x != x2.x && x1.y != x2.y);
            float mass = 2.0f * m1 / (m1 + m2);
            Vector2 v21 = v2 - v1, x21 = x2 - x1;
            float x21_length = x21.magnitude;
            float d = Vector2.Dot(v21, x21);

            float temp = mass * d / (x21_length * x21_length);
            x21.x = x21.x * temp;
            x21.y = x21.y * temp;

            return v2 - x21;
        }
		
		
        public CollisionVectors momentum_angle_free(float v1X, float v1Y, float v2X, float v2Y, float m1,
            float m2, float x1X,
            float x1Y, float x2X, float x2Y)
        {
            CollisionVectors vec;
            Vector2 v1 = new Vector2(v1X, v1Y), v2 = new Vector2(v2X, v2Y);
            Vector2 x1 = new Vector2(x1X, x1Y), x2 = new Vector2(x2X, x2Y);
            Vector2 v12 = v1_vector_components(v1, v2, m1, m2, x1, x2); 
            Vector2 v21 = v2_vector_components(v1, v2, m1, m2, x1, x2);
            vec.V12 = v12;
            vec.V21 = v21;

            return vec;
        }
		
		// return Object 1 & Object 2 final velocities and directions after contact (use structural objects)
        public CollisionVectors momentum_angle_free1(ColliderObject obj1, ColliderObject obj2)
        {
            Vector2 v12, v21;
            CollisionVectors vec;
            v12 = v1_vector_components(obj1.Vector, obj2.Vector, obj1.Mass, obj2.Mass, obj1.Centre, obj2.Centre);
            v21 = v2_vector_components(obj1.Vector, obj2.Vector, obj1.Mass, obj2.Mass, obj1.Centre, obj2.Centre);

            vec.V12 = v12;
            vec.V21 = v21;

            return vec;
        }
    }
}
