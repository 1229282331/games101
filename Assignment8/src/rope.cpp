#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D step = (end-start) / (num_nodes-1);
        for(int i=0; i<num_nodes; i++)
        {
            Mass* mass = new Mass(start+step*i, node_mass, false);
            mass->velocity = Vector2D(0.0, 0.0);
            mass->forces = Vector2D(0.0, 0.0);
            masses.push_back(mass); 
            if(i>0)
            {
                Spring* spring = new Spring(masses[i-1], masses[i], k);
                springs.push_back(spring);
            }
        }
        for(auto i : pinned_nodes)
            masses[i]->pinned = true;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        double kd = 3e-3;
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D b = s->m1->position, a = s->m2->position;
            s->m1->forces += -s->k * (b-a)/(b-a).norm() * ((b-a).norm()-s->rest_length);
            s->m2->forces += -s->k * (a-b)/(b-a).norm() * ((b-a).norm()-s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                Vector2D a = m->forces/m->mass + gravity - kd*m->velocity/m->mass;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        double dampling_factor = 1e-7;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D b = s->m1->position, a = s->m2->position;
            s->m1->forces += -s->k * (b-a)/(b-a).norm() * ((b-a).norm()-s->rest_length);
            s->m2->forces += -s->k * (a-b)/(b-a).norm() * ((b-a).norm()-s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D a = m->forces/m->mass + gravity;
                m->position = temp_position + (1.0-dampling_factor)*(temp_position-m->last_position) + a*delta_t*delta_t;
                m->last_position = temp_position; 
                // TODO (Part 4): Add global Verlet damping
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }

    }
}
