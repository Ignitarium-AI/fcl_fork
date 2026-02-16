#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision_object.h>
#include <GL/freeglut.h>
#include <iostream>
#include <memory>
#include <vector>

using namespace fcl;

// Global FCL objects so OpenGL callbacks can access them
std::shared_ptr<Boxf> box1_geom;
std::shared_ptr<Boxf> box2_geom;
CollisionObjectf* obj1;
CollisionObjectf* obj2;
std::vector<Contactf> contact_points; // Store collision contact points

// Helper function to draw a wireframe box
void drawBox(float size_x, float size_y, float size_z) {
    glPushMatrix();
    glScalef(size_x, size_y, size_z);
    glutWireCube(1.0);
    glPopMatrix();
}

// OpenGL Display Callback
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Set up a basic camera looking at the origin
    gluLookAt(3.0, 2.0, 3.0,   // Camera position (X, Y, Z)
              0.0, 0.0, 0.0,   // Look-at target
              0.0, 1.0, 0.0);  // Up vector

    // 1. Draw Box 1 (Blue)
    glPushMatrix();
    Eigen::Matrix4f mat1 = obj1->getTransform().matrix();
    glMultMatrixf(mat1.data()); // Apply FCL transform to OpenGL
    glColor3f(0.2f, 0.5f, 1.0f);
    drawBox(1.0, 1.0, 1.0);
    glPopMatrix();

    // 3. Draw Box 2 (Red)
    glPushMatrix();
    Eigen::Matrix4f mat2 = obj2->getTransform().matrix();
    glMultMatrixf(mat2.data()); // Apply FCL transform to OpenGL
    glColor3f(1.0f, 0.2f, 0.2f);
    drawBox(1.0, 1.0, 1.0);
    glPopMatrix();

    // 3. Draw Contact Points (Yellow Spheres)
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow color
    for (const auto& contact : contact_points) {
        glPushMatrix();
        glTranslatef(contact.pos.x(), contact.pos.y(), contact.pos.z());
        glutSolidSphere(0.05, 16, 16); // Small sphere with radius 0.05
        glPopMatrix();
    }

    glutSwapBuffers();
}

// OpenGL Reshape Callback to handle window resizing
void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)w / (double)h, 1.0, 100.0);
}

int main(int argc, char** argv) {
    // --- FCL SETUP ---
    box1_geom = std::make_shared<Boxf>(1.0, 1.0, 1.0);
    box2_geom = std::make_shared<Boxf>(1.0, 1.0, 1.0);

    Transform3f tf1 = Transform3f::Identity();
    tf1.translation() << 0, 0, 0;

    Transform3f tf2 = Transform3f::Identity();
    tf2.translation() << 0.8, 0.2, 0.0; // Offset it slightly so they intersect

    obj1 = new CollisionObjectf(box1_geom, tf1);
    obj2 = new CollisionObjectf(box2_geom, tf2);

    // --- OPENGL SETUP ---
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("FCL Collision Visualizer");

    glEnable(GL_DEPTH_TEST); // Enable depth so 3D renders correctly
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f); // Dark grey background

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    // --- FCL COLLISION TEST ---
    CollisionRequestf request;
    request.enable_contact = true;     // Tell FCL we want the exact contact points
    request.num_max_contacts = 10;     // Maximum number of contacts to calculate
    CollisionResultf result;
    
    collide(obj1, obj2, request, result);
    
    // Print collision results
    if (result.isCollision()) {
        std::cout << "Collision detected!" << std::endl;
        result.getContacts(contact_points);
        std::cout << "Number of contacts: " << contact_points.size() << std::endl;
        for (size_t i = 0; i < contact_points.size(); ++i) {
            std::cout << "Contact " << i << " position: ("
                      << contact_points[i].pos.x() << ", "
                      << contact_points[i].pos.y() << ", "
                      << contact_points[i].pos.z() << ")" << std::endl;
        }
    } else {
        std::cout << "No collision detected." << std::endl;
    }

    std::cout << "Starting visualization. Close the window to exit." << std::endl;
    
    // Start the GUI loop
    glutMainLoop();

    // Cleanup FCL objects
    delete obj1;
    delete obj2;

    return 0;
}
