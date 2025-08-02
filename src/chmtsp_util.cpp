#include "chmtsp.hpp"
#include "chmtsp_util.hpp"
#include <cstddef>
#include <sstream>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>


std::vector<size_t> unassigned_nodes;          // vetor de nós não assinalados
int n_nodes;                                // número de nós
int k_vehicles;                             // número de veículos
int r_radius;                               // raio de cobertura
typedef struct Coord  {                     // strutura de coordenadas
    double pos_x;
    double pos_y;
} Coord ;
std::vector<Coord> coordinates;             // coordenadas dos nós
std::vector<std::vector<int>> matriz_dist;  // matriz de adjacencias
typedef struct Nodes {
    size_t index;
    Coord pos;
} Nodes;                                    // nodes structure


int read_instance(std::string file_name) {
    // create fstream object from file name string
    std::ifstream input_instance(file_name);
    bool parameters_read {false};

    // checks if it was open
    if(!input_instance.is_open()) {
        std::cerr << "error: could not open file" << std::endl;
        return 1;
    }

    std::string line;                   // string para ler as linhas
    while (std::getline(input_instance, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);     // stream para ler os valores separados por espaços

        if (!parameters_read) {
            ss >> k_vehicles >> n_nodes >> r_radius;
            n_nodes++;                  // adiciona garagem ao número de nós
            matriz_dist.resize(n_nodes, std::vector<int>(n_nodes));     // aloca matriz de distancias
            for (int i{}; i < n_nodes; i++) {
                unassigned_nodes.push_back(i);                          // coloca todos os nós como não assinalados
            }
            parameters_read = true;
        } else {
            Coord new_coord;
            ss >> new_coord.pos_x >> new_coord.pos_y;
            coordinates.push_back(new_coord);
        }
    }
    input_instance.close();
    return 0;
}

int read_inst_dist(std::string input_file) {

    // create fstream file object from file name string
    std::ifstream input_instance(input_file);

    // checks if the file was open
    if (!input_instance.is_open()) {
        std::cerr << "error: could not open file" << std::endl;
        return 1;
    }

    bool parameters_read {false};
    std::string line;

    while (std::getline(input_instance, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);     // stream para ler os valores separados por espaços

        if(!parameters_read) {
            parameters_read = true;
        } else {
            size_t i{};
            size_t j{};
            int dist{};
            ss >> i >> j >> dist;
            matriz_dist[i][j] = dist;   // le matriz de distancias
        }

    }

    return 0;
}


int orientation(Coord a, Coord b, Coord c) {
    long long area = (b.pos_x - a.pos_x)*(c.pos_y - a.pos_y) - (c.pos_x - a.pos_x)*(b.pos_y - a.pos_y);
    if (area < 0) {
        return -1; // cw
    } else if (area > 0) {
        return 1; // ccw
    }
    return 0;
}

int distance(const Nodes& a, const Nodes& b) {
    double dx = a.pos.pos_x - b.pos.pos_x;
    double dy = a.pos.pos_y - b.pos.pos_y;
    double result = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    return std::round(result);
}


int save_data(const std::vector<int>& hull_indices) {
    std::ofstream points_file("points.dat");
    for (const auto& coord : coordinates) {
        points_file << coord.pos_x << " " << coord.pos_y << "\n";
    }
    points_file.close();

    std::ofstream hull_file("hull.dat");
    for (int idx : hull_indices) {
        hull_file << coordinates[idx].pos_x << " " << coordinates[idx].pos_y << "\n";
    }
    // Close the hull by repeating the first point
    hull_file << coordinates[hull_indices[0]].pos_x << " " << coordinates[hull_indices[0]].pos_y << "\n";
    hull_file.close();
    return 0;
}

std::vector<int> find_initial_hull() {
    // Find the leftmost unassigned node
    size_t point_left_most_i = unassigned_nodes[0];
    for (size_t idx = 1; idx < unassigned_nodes.size(); idx++) {
        Coord left_most_coord = coordinates[point_left_most_i];
        Coord current_point = coordinates[unassigned_nodes[idx]];
        if (current_point.pos_x < left_most_coord.pos_x ||
            (current_point.pos_x == left_most_coord.pos_x &&
             current_point.pos_y < left_most_coord.pos_y)) {
            point_left_most_i = unassigned_nodes[idx];
        }
    }

    // Build vector of Nodes for unassigned nodes (excluding leftmost)
    std::vector<Nodes> nodes;
    Nodes point_left_most;
    point_left_most.index = point_left_most_i;
    point_left_most.pos = coordinates[point_left_most_i];
    for (size_t i = 0; i < unassigned_nodes.size(); i++) {
        size_t idx = unassigned_nodes[i];
        if (idx != point_left_most_i) {
            Nodes n;
            n.index = idx;
            n.pos = coordinates[idx];
            nodes.push_back(n);
        }
    }

    // Sort nodes by polar angle with respect to leftmost point
    std::sort(nodes.begin(), nodes.end(), [&point_left_most](const Nodes& a, const Nodes& b) {
        int d = orientation(point_left_most.pos, a.pos, b.pos);
        if (d > 0) return true;
        if (d < 0) return false;
        // Collinear: closer one first
        return distance(point_left_most, a) < distance(point_left_most, b);
    });

    // Graham scan
    std::vector<Nodes> stack;
    stack.push_back(point_left_most);
    if (!nodes.empty()) stack.push_back(nodes[0]);
    for (size_t i = 1; i < nodes.size(); i++) {
        while (stack.size() > 1 &&
               orientation(stack[stack.size()-2].pos, stack[stack.size()-1].pos, nodes[i].pos) <= 0) {
            stack.pop_back();
        }
        stack.push_back(nodes[i]);
    }

    // Return the indices of the hull
    std::vector<int> hull_indices;
    for (const auto& n : stack) {
        hull_indices.push_back(static_cast<int>(n.index));
    }
    save_data(hull_indices);
    return hull_indices;
}
