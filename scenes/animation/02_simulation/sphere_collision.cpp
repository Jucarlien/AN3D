
#include "sphere_collision.hpp"

#include <random>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;




void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    create_new_particle();
    compute_time_step(dt, scene);

    display_particles(scene);
    draw(borders, scene.camera);
}

void scene_model::compute_time_step(float dt, scene_structure& scene)
{
    float epsilon = 0.01;
    // Set forces
    const size_t N = particles.size();


    // Collisions with cube
    // ... to do
    static const std::vector<vec3> normals = {{1,0,0},{0,1,0},{0,0,1},{-1,0,0},{0,-1,0},{0,0,-1}};
    static const std::vector<vec3> points = {{-1,0,0},{0,-1,0},{0,0,-1},{1,0,0},{0,1,0},{0,0,1}};

    for(size_t k1=0; k1 < N; k1++)
    {
        particle_structure& sphere = particles[k1];
        vec3& v1 = sphere.v;
        vec3& p1 = sphere.p;
        auto r1 = sphere.r;
        float friction = 0.7;



   



        //SPHERE COLLISION 
        for (size_t k2 = k1 + 1; k2 < N; k2++)
        {
            particle_structure& sphere2 = particles[k2];
            vec3& v2 = sphere2.v;
            auto r2 = sphere2.r;
            auto p2 = sphere2.p;

            //auto pnorm = std::sqrt(dot(p1 - p2, p1 - p2));
            auto u = (p1 - p2) / norm(p1 - p2);
            auto v1n = dot(v1, u) * u;
            auto v2n = dot(v2, -u) * -u;
            auto v1t = v1 - (v1n);
            auto v2t = v2 - (v2n);

            if (norm(p1 - p2) <= r1 + r2)
            {
                //v1 = 0.9 * v1t + 0.9 * v2n;
                //v2 = 0.9 * v2t + 0.9 * v1n;
                if (norm(v2) + norm(v1) < 0.1) {
                    //v2 *= friction;
                    v1 *= friction;
                    v2 *= friction;
                    continue;
                }
                v1 += friction * (dot(v2 - v1, u) * u);
                v2 -= friction * (dot(v2 - v1, u) * u);
                float d = r1 + r2 - norm(p1 - p2);
                sphere.p += (d / 2) *  u;
                //sphere.p += d * u;
            }
        }


        if (norm(v1) < 0.0001)
        {
            friction = 0.1;
        }
        //BORDERS COLLISION
        for(size_t i = 0; i < 6; i++)
        {
            auto n = normals[i];
            auto pi = sphere.p;
            float detection = dot((pi - points[i]), n);
            if (detection <= sphere.r) {
                float vn = dot(v1, n);
                //vec3 vt = v - (vn * n);
                v1 = (v1 - (2 * vn * n)) * friction;
                float d = sphere.r - detection;
                sphere.p += d * n;
                //v = -vn * n + vt;
            } 
        }

        //GRAVITY
        sphere.f = vec3(0,-9.81f,0);
        sphere.f = update_gravity(scene);
        vec3 const& f = sphere.f;

        v1 = (1-0.9*dt) * v1 + dt * f; // gravity + friction force
        //v1 *= friction;
        p1 = p1 + dt * v1;


    }

    // Collisions between spheres
    // ... to do

    //update_translation(scene);
}



float prev_a = 100000000;
vec3 grav = {0,0,0};


vec3 scene_model::update_gravity(scene_structure& scene){
    auto mat = scene.camera.orientation;
    float g = -9.81;

    float x = mat[1];
    float y = mat[4];
    float z = mat[7];
    
    grav = vec3(x * g, y * g, z * g);

    return grav;

}

vec3 scene_model::update_translation(scene_structure& scene){
    //vec3 trans = scene.camera.translation;
    //std::cout<< trans << "\n\n";
}


vec3 scene_model::determine_gravity(scene_structure& scene) {
    float a = gui_scene.gravity;
    if (a == prev_a){

        return grav;
    }
    prev_a = a;
    float g = -9.81;
    float y = g * a;
    
    float temp = 1 - a;
    float x = 0;
    if (a / 90 < -1)
    {
         x = (a + 90) / 90;
        y = -1 - x;
    }

    else if (a / 90 < 0)
    {
        y = a / 90;
        x = 1 + y;
    }
    
    else if (a / 90 > 1)
    {
        x = -(a - 90) / 90;
        y = 1 + x;
    }

    else if (a / 90 >= 0)
    {
        y = (a / 90);
        x = 1 - y;
    }
    grav = vec3(x * g, 0, y * g);
    scene.camera.apply_rotation(0.1, 0.0, 0.0, 0);
    return grav;
}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);

    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
    draw(cube, scene.camera);
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

    cube = mesh_drawable( mesh_primitive_parallelepiped(vec3(0,0,0),vec3(0.5,0.5,0),vec3(0,0.5,0.5),vec3(0.5,0,0.5)));
    //cube = mesh_drawable(mesh_primitive_cone(0.1f, {0,0,0,},{0,0.2f,0}));
    cube.uniform.color = {50,50,50};
    cube.shader = shaders["mesh"];
}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);
//    
    ImGui::SliderFloat("rotation", &gui_scene.gravity, -180.0f,180.0f, "%.2f G");
//
    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}





#endif
