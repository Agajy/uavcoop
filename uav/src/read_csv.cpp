#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include "read_csv.h"

std::vector<std::pair<double, double>> readcsv() {
    std::ifstream file("output2.csv"); // Ouvrir le fichier CSV
    std::vector<std::pair<double, double>> coordinates;
    if (!file) {
        std::cerr << "Erreur: Impossible d'ouvrir le fichier!" << std::endl;
        return coordinates;
    }

    std::string line;

    // Lire chaque ligne du fichier
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;

        // Lire les deux premières valeurs (x et y)
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            coordinates.emplace_back(x, y);
        }
    }

    file.close(); // Fermer le fichier

    // Afficher les coordonnées x, y
    for (const auto& coord : coordinates) {
        std::cout << "x: " << coord.first << ", y: " << coord.second << std::endl;
    }

    return coordinates;
}
