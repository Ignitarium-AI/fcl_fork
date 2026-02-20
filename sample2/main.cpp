#include <fcl/fcl.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <cmath>
#include <omp.h> // Compile with -fopenmp
#include <random>
#include <iomanip>

using namespace fcl;

// =========================================================
// PART 1: The Core Physics (Separating Axis Theorem)
// *** FPGA IMPLEMENTATION TARGET - HIGH PRIORITY ***
// Lines 17-60: Core SAT algorithm - ideal for FPGA pipelines
// Fixed-point arithmetic, parallel projections, early termination
// =========================================================

// Projects both triangles onto a specific axis and checks for an overlap gap.
// *** FPGA NOTE: Dot products and min/max operations perfect for parallel units ***
bool checkOverlapOnAxis(const Vector3d& axis, const Vector3d U[3], const Vector3d V[3]) {
    if (axis.squaredNorm() < 1e-8) return true; // Ignore zero-vectors

    double minU = axis.dot(U[0]), maxU = minU;
    for (int i = 1; i < 3; ++i) {
        double projection = axis.dot(U[i]);
        minU = std::min(minU, projection);
        maxU = std::max(maxU, projection);
    }

    double minV = axis.dot(V[0]), maxV = minV;
    for (int i = 1; i < 3; ++i) {
        double projection = axis.dot(V[i]);
        minV = std::min(minV, projection);
        maxV = std::max(maxV, projection);
    }

    if (maxU < minV || maxV < minU) return false; // Gap found, no intersection
    return true; 
}

// 11-Axis SAT Triangle-Triangle Intersection
// *** FPGA NOTE: This function should be highly optimized - core collision kernel ***
// Independent for each triangle pair, enables massive parallelization
bool TriTriIntersect(const Vector3d& U0, const Vector3d& U1, const Vector3d& U2,
                     const Vector3d& V0, const Vector3d& V1, const Vector3d& V2) {
    Vector3d U[3] = {U0, U1, U2};
    Vector3d V[3] = {V0, V1, V2};

    Vector3d edgeU[3] = { U1 - U0, U2 - U1, U0 - U2 };
    Vector3d edgeV[3] = { V1 - V0, V2 - V1, V0 - V2 };

    Vector3d normalU = edgeU[0].cross(edgeU[1]);
    if (!checkOverlapOnAxis(normalU, U, V)) return false;

    Vector3d normalV = edgeV[0].cross(edgeV[1]);
    if (!checkOverlapOnAxis(normalV, U, V)) return false;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3d crossAxis = edgeU[i].cross(edgeV[j]);
            if (!checkOverlapOnAxis(crossAxis, U, V)) return false;
        }
    }
    return true; 
}

// =========================================================
// PART 2: Atay & Bayazit Logic (Brute Force + Broad Phase)
// *** FPGA IMPLEMENTATION TARGET - MEDIUM PRIORITY ***
// Lines 57-86: AABB calculations ideal for parallel min/max units
// Lines 110-129: Main collision loop - perfect for FPGA parallelization
// =========================================================

// Primitive Broad-Phase: Calculates a dynamic Axis-Aligned Bounding Box (AABB) 
// *** FPGA NOTE: Parallel min/max reduction, perfect memory access pattern ***
bool PrimitiveBroadPhase(const BVHModel<OBBRSSd>* m1, const Transform3d& tf1,
                         const BVHModel<OBBRSSd>* m2, const Transform3d& tf2) {
    
    if (m1->num_vertices == 0 || m2->num_vertices == 0) return false;

    // Calculate AABB for Object 1
    Vector3d min1 = tf1 * m1->vertices[0];
    Vector3d max1 = min1;
    for (int i = 1; i < m1->num_vertices; ++i) {
        Vector3d v = tf1 * m1->vertices[i];
        min1[0] = std::min(min1[0], v[0]); max1[0] = std::max(max1[0], v[0]);
        min1[1] = std::min(min1[1], v[1]); max1[1] = std::max(max1[1], v[1]);
        min1[2] = std::min(min1[2], v[2]); max1[2] = std::max(max1[2], v[2]);
    }

    // Calculate AABB for Object 2
    Vector3d min2 = tf2 * m2->vertices[0];
    Vector3d max2 = min2;
    for (int i = 1; i < m2->num_vertices; ++i) {
        Vector3d v = tf2 * m2->vertices[i];
        min2[0] = std::min(min2[0], v[0]); max2[0] = std::max(max2[0], v[0]);
        min2[1] = std::min(min2[1], v[1]); max2[1] = std::max(max2[1], v[1]);
        min2[2] = std::min(min2[2], v[2]); max2[2] = std::max(max2[2], v[2]);
    }

    // Check for overlap along the X, Y, and Z axes
    if (max1[0] < min2[0] || max2[0] < min1[0]) return false; 
    if (max1[1] < min2[1] || max2[1] < min1[1]) return false; 
    if (max1[2] < min2[2] || max2[2] < min1[2]) return false; 

    return true; 
}

// The parallelized narrow-phase checker
// *** FPGA NOTE: Replace OpenMP with hardware parallelization ***
// This nested loop structure maps directly to FPGA processing arrays
bool AtayBayazitCollide(const BVHModel<OBBRSSd>* m1, const Transform3d& tf1,
                        const BVHModel<OBBRSSd>* m2, const Transform3d& tf2) {
    
    // 1. HARDWARE-STYLE BROAD PHASE
    // *** FPGA NOTE: Implement as parallel AABB units ***
    if (!PrimitiveBroadPhase(m1, tf1, m2, tf2)) {
        return false; 
    }

    // 2. PARALLEL NARROW PHASE (Only runs if broad-phase overlaps)
    // *** FPGA NOTE: Main parallelization target - each triangle pair independent ***
    bool collision = false;
    
    // *** FPGA IMPLEMENTATION TARGET: Replace with hardware parallel units ***
    // Each (i,j) iteration can run on separate FPGA processing elements
    #pragma omp parallel for collapse(2) shared(collision)
    for (int i = 0; i < m1->num_tris; ++i) {
        for (int j = 0; j < m2->num_tris; ++j) {
            // *** FPGA NOTE: Early exit logic - implement with global collision flag ***
            if (collision) continue; // Early exit if another thread found a collision

            // *** FPGA NOTE: Triangle data access - optimize memory controllers ***
            Triangle tri1 = m1->tri_indices[i];
            Triangle tri2 = m2->tri_indices[j];

            // *** FPGA NOTE: Transform operations - pipeline these calculations ***

            Vector3d U0 = tf1 * m1->vertices[tri1[0]];
            Vector3d U1 = tf1 * m1->vertices[tri1[1]];
            Vector3d U2 = tf1 * m1->vertices[tri1[2]];

            Vector3d V0 = tf2 * m2->vertices[tri2[0]];
            Vector3d V1 = tf2 * m2->vertices[tri2[1]];
            Vector3d V2 = tf2 * m2->vertices[tri2[2]];

            // *** FPGA NOTE: Core collision kernel call - highest optimization priority ***
            if (TriTriIntersect(U0, U1, U2, V0, V1, V2)) {
                collision = true; // *** FPGA NOTE: Global collision flag update ***
            }
        }
    }
    return collision;
}

// =========================================================
// PART 3: High-Resolution Mesh Generator
// =========================================================

std::shared_ptr<BVHModel<OBBRSSd>> createSphereMesh(double radius, int stacks, int slices) {
    std::vector<Vector3d> vertices;
    std::vector<Triangle> triangles;

    for (int i = 0; i <= stacks; ++i) {
        double phi = M_PI * i / stacks;
        for (int j = 0; j <= slices; ++j) {
            double theta = 2.0 * M_PI * j / slices;
            double x = radius * sin(phi) * cos(theta);
            double y = radius * sin(phi) * sin(theta);
            double z = radius * cos(phi);
            vertices.emplace_back(x, y, z);
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = (i * (slices + 1)) + j;
            int second = first + slices + 1;
            
            triangles.emplace_back(first, second, first + 1);
            triangles.emplace_back(second, second + 1, first + 1);
        }
    }

    auto model = std::make_shared<BVHModel<OBBRSSd>>();
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();
    return model;
}

// =========================================================
// PART 4: The Unbiased, 3-Scenario Benchmark
// =========================================================

int main() {
    // Generate Random Parameters
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> mesh_res_dist(20, 40); 
    std::uniform_real_distribution<> radius_dist(1.0, 2.0); 

    int stacks = mesh_res_dist(gen);
    int slices = mesh_res_dist(gen);
    double radius = radius_dist(gen);
    
    std::cout << "==================================================\n";
    std::cout << "--- Generative Mesh Parameters ---\n";
    std::cout << "Radius: " << radius << " | Stacks: " << stacks << " | Slices: " << slices << "\n";

    auto shared_mesh = createSphereMesh(radius, stacks, slices);
    long total_pairs = (long)shared_mesh->num_tris * (long)shared_mesh->num_tris;
    
    std::cout << "Triangles per object: " << shared_mesh->num_tris << "\n";
    std::cout << "Worst-Case Brute Force Checks: " << total_pairs << " pairs.\n";
    std::cout << "==================================================\n\n";

    CollisionObjectd obj1(shared_mesh);
    CollisionObjectd obj2(shared_mesh);

    // Configure FCL for pure Boolean checks
    CollisionRequest<double> request;
    request.enable_contact = false;
    request.num_max_contacts = 1;   
    
    // Benchmarking Helper Lambda
    auto runBenchmark = [&](const std::string& scenario_name, const Vector3d& translation) {
        std::cout << ">>> SCENARIO: " << scenario_name << " <<<\n";
        
        Transform3d tf1 = Transform3d::Identity();
        Transform3d tf2 = Transform3d::Identity();
        tf2.translation() = translation;
        
        obj1.setTransform(tf1);
        obj2.setTransform(tf2);

        // --- Atay & Bayazit Brute Force ---
        auto start_atay = std::chrono::high_resolution_clock::now();
        bool res_atay = AtayBayazitCollide(shared_mesh.get(), tf1, shared_mesh.get(), tf2);
        auto end_atay = std::chrono::high_resolution_clock::now();
        double time_atay = std::chrono::duration<double, std::micro>(end_atay - start_atay).count();

        // --- FCL Optimized ---
        CollisionResult<double> result;
        auto start_fcl = std::chrono::high_resolution_clock::now();
        fcl::collide(&obj1, &obj2, request, result);
        auto end_fcl = std::chrono::high_resolution_clock::now();
        double time_fcl = std::chrono::duration<double, std::micro>(end_fcl - start_fcl).count();

        // Check for method agreement and show appropriate result
        bool methods_agree = (res_atay == result.isCollision());
        if (methods_agree) {
            std::cout << "Result: " << (result.isCollision() ? "[COLLISION]" : "[SAFE]") << "\n";
        } else {
            std::cout << "Result: FCL: " << (result.isCollision() ? "[COLLISION]" : "[SAFE]") 
                      << " | Atay: " << (res_atay ? "[COLLISION]" : "[SAFE]") << " (METHODS DISAGREE)\n";
        }
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Atay (Broad+Narrow): " << time_atay << " us\n";
        std::cout << "FCL (BVH Tree):      " << time_fcl << " us\n";
        
        if (time_atay > time_fcl) {
            std::cout << "Winner: FCL by " << (time_atay / time_fcl) << "x\n\n";
        } else {
            std::cout << "Winner: Atay by " << (time_fcl / time_atay) << "x\n\n";
        }
    };

    // ----------------------------------------------------------------
    // SCENARIO 1: Far Apart
    // ----------------------------------------------------------------
    Vector3d far_apart(radius * 2.5, 0, 0);
    runBenchmark("1. Far Apart (No AABB Overlap)", far_apart);

    // ----------------------------------------------------------------
    // SCENARIO 2: The Close Call
    // ----------------------------------------------------------------
    double offset = radius * 1.2; 
    Vector3d close_call(offset, offset, offset);
    runBenchmark("2. Close Call (Methods May Disagree)", close_call);

    // ----------------------------------------------------------------
    // SCENARIO 3: Deep Intersection
    // ----------------------------------------------------------------
    Vector3d deep_intersect(radius * 0.5, 0, 0);
    runBenchmark("3. Deep Intersection", deep_intersect);

    return 0;
}